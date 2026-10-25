# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Label-text channel: what a reader saw, scored against every registered product's label text."""
# The second rung of the identity ladder (CMB-112 round 3). The registry holds each product's
# printed text (``ProductRecord.ocr_strings``) and, derived from its registered views, what the OCR
# engine itself reads on them. A reader's output for one object is scored against *every* product:
#
#   score(P) = sum of w(word) over P's words that were read / sum of w(word) over P's words
#
# where a word is "read" if some read token is close to it (difflib ratio) and w is an inverse
# document frequency over the registered products, so a flavour word that only one product prints
# counts more than a brand or a kind that several print. The answer is one product only if it
# clears a minimum score *and* beats the runner-up by a clear margin: two look-alikes that share
# brand and kind both score the same on the shared words and neither is named until the word that
# tells them apart is read. That margin, not a vote, is the tie-breaker the embedding lacks.
#
# Nothing here is trained; every number comes from the registry and the reader's own output.

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
import difflib
import functools
import math
import re

_WORD = re.compile(r"[a-z0-9]+")
MIN_WORD_LENGTH = 3
MIN_CONTRADICTION_LENGTH = 4


@functools.lru_cache(maxsize=500_000)
def similarity(a: str, b: str) -> float:
    """difflib ratio of two words (cached: scoring compares every token with every word)."""
    return difflib.SequenceMatcher(None, a, b).ratio()


def token_reads_word(word: str, token: str, min_ratio: float = 0.8) -> bool:
    """Does the read ``token`` stand for the registered ``word``?

    Exact; or the word inside a longer token (a reader often joins neighbouring words,
    "fennlomelon", and a word of four letters or more is specific enough to be found inside
    one); or close in spelling and in length (a four-letter "cool" must not read the six-letter
    "cooler"); or a close window of a joined token.
    """
    if word == token:
        return True
    n, m = len(word), len(token)
    # A joined token is at most about two words long: a long garbled run that merely contains a
    # short word ("juiceconcentrae") is not that word.
    if n >= 4 and word in token and m <= 2 * n + 2:
        return True
    # A long word read in part ("anova" of "talanova"): most of it, and specific enough.
    if m >= 5 and token in word and m >= 0.6 * n:
        return True
    if 0.75 * n <= m <= 1.35 * n + 1 and similarity(word, token) >= min_ratio:
        return True
    if n >= 5 and n + 1 < m <= 2 * n + 2:
        return any(
            similarity(word, token[i : i + n]) >= max(min_ratio, 0.85) for i in range(m - n + 1)
        )
    return False


def words(text: str) -> list[str]:
    return _WORD.findall(text.lower())


def label_words(strings: Iterable[str]) -> tuple[str, ...]:
    """A product's distinct label words (at least three characters), in print order."""
    seen: list[str] = []
    for text in strings:
        for word in words(text):
            if len(word) >= MIN_WORD_LENGTH and word not in seen:
                seen.append(word)
    return tuple(seen)


@dataclass(frozen=True)
class TextScore:
    product_id: str
    score: float
    matched: tuple[str, ...]
    missing: tuple[str, ...]
    # Read tokens of at least MIN_CONTRADICTION_LENGTH letters that stand for a word some *other*
    # registered product prints and for none of this product's: printed text the product lacks.
    contradicted_by: tuple[str, ...] = ()


@dataclass(frozen=True)
class TextMatch:
    """Every product's score on one object's reads, best first."""

    ranked: tuple[TextScore, ...]
    n_tokens: int

    @property
    def best(self) -> TextScore | None:
        return self.ranked[0] if self.ranked else None

    @property
    def runner_up(self) -> TextScore | None:
        return self.ranked[1] if len(self.ranked) > 1 else None

    @property
    def margin(self) -> float:
        if self.best is None:
            return 0.0
        return self.best.score - (self.runner_up.score if self.runner_up else 0.0)


@dataclass(frozen=True)
class TextPolicy:
    """When a text match names a product. Fixed beforehand or fitted on train/validation only."""

    min_score: float = 0.5
    min_margin: float = 0.25
    min_matched: int = 2
    # A product is not named while more than this many read tokens contradict it (None = ignore).
    max_contradictions: int | None = 0

    def names(self, match: TextMatch) -> TextScore | None:
        best = match.best
        if best is None or best.score < self.min_score or match.margin < self.min_margin:
            return None
        if len(best.matched) < self.min_matched:
            return None
        if (
            self.max_contradictions is not None
            and len(best.contradicted_by) > self.max_contradictions
        ):
            return None
        return best


class LabelTextIndex:
    """Registered label words per product, and the scoring of a reader's tokens against them."""

    def __init__(
        self,
        strings_by_product: Mapping[str, Sequence[str]],
        *,
        min_ratio: float = 0.8,
    ) -> None:
        self.min_ratio = min_ratio
        self._words = {p: label_words(s) for p, s in strings_by_product.items()}
        owners: dict[str, int] = {}
        for product_words in self._words.values():
            for word in product_words:
                owners[word] = owners.get(word, 0) + 1
        n = max(len(self._words), 1)
        self._weight = {w: 1.0 + math.log(n / count) for w, count in owners.items()}
        self._vocabulary = tuple(owners)

    @property
    def product_ids(self) -> tuple[str, ...]:
        return tuple(self._words)

    def words_of(self, product_id: str) -> tuple[str, ...]:
        return self._words[product_id]

    def weight(self, word: str) -> float:
        return self._weight.get(word, 1.0)

    def reads(self, word: str, tokens: Sequence[str]) -> bool:
        """Does some read token stand for ``word``?"""
        return any(token_reads_word(word, token, self.min_ratio) for token in tokens)

    def tokens_of(self, read_lines: Iterable[str]) -> list[str]:
        """Distinct read tokens (any length of two or more characters), in read order."""
        seen: list[str] = []
        for line in read_lines:
            for token in words(line):
                if len(token) >= 2 and token not in seen:
                    seen.append(token)
        return seen

    def score(self, tokens: Sequence[str], *, restrict: Iterable[str] | None = None) -> TextMatch:
        """Score ``tokens`` against every registered product (or only ``restrict``)."""
        pool = list(restrict) if restrict is not None else list(self._words)
        scores: list[TextScore] = []
        for product_id in pool:
            product_words = self._words[product_id]
            if not product_words:
                continue
            read = [w for w in product_words if self.reads(w, tokens)]
            missing = tuple(w for w in product_words if w not in read)
            total = sum(self.weight(w) for w in product_words)
            score = sum(self.weight(w) for w in read) / total
            contradicted = tuple(
                t
                for t in tokens
                if len(t) >= MIN_CONTRADICTION_LENGTH
                and t.isalpha()
                and not any(token_reads_word(w, t, self.min_ratio) for w in product_words)
                and any(token_reads_word(w, t, self.min_ratio) for w in self._vocabulary)
            )
            scores.append(TextScore(product_id, score, tuple(read), missing, contradicted))
        scores.sort(key=lambda s: (-s.score, len(s.missing), s.product_id))
        return TextMatch(tuple(scores), len(tokens))
