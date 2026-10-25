# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""OCR channel: match what a reader saw against the registry's OCR strings, conservatively."""
# CMB-101 measured OCR as 100 percent precise when it answers and answering on about a quarter
# of crops. Precision is the whole point of this channel, so the rule is strict:
#
# * a registered word counts as read only if a read word is close to it (difflib ratio) and the
#   registered word is at least three characters long;
# * a product is *named* only if every one of its distinguishing words was read, where
#   distinguishing means the word appears in no other registered product (the flavour, in a
#   catalogue of brand-and-flavour look-alikes; every word, for a product with no registered
#   twin), and at least two words of the product were read in all;
# * if more than one product satisfies that, or several crops name different products, the
#   channel stays silent about the conflict (it reports it, the fusion abstains).
#
# The second clause is what keeps a held-out look-alike ("Dalbex Cherry Fizz" against a
# registered "Dalbex Apple Fizz") from being named by its shared brand and kind alone.

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
import difflib
import re

_WORD = re.compile(r"[a-z0-9]+")
MIN_WORD_LENGTH = 3


def words(text: str) -> list[str]:
    return _WORD.findall(text.lower())


@dataclass(frozen=True)
class OcrAnswer:
    """What the OCR channel concluded for one or several crops."""

    product_id: str | None
    # Registered words matched for the named product (empty when silent).
    matched: tuple[str, ...]
    # Every product that satisfied the naming rule on any crop (len > 1 means a conflict).
    candidates: tuple[str, ...]
    raw_words: int

    @property
    def conflict(self) -> bool:
        return len(self.candidates) > 1


class OcrMatcher:
    """Matches OCR reads to registered products by their OCR strings."""

    def __init__(
        self, strings_by_product: dict[str, Sequence[str]], *, min_ratio: float = 0.8
    ) -> None:
        self.min_ratio = min_ratio
        self._words: dict[str, tuple[str, ...]] = {}
        for product_id, strings in strings_by_product.items():
            seen: list[str] = []
            for text in strings:
                for word in words(text):
                    if len(word) >= MIN_WORD_LENGTH and word not in seen:
                        seen.append(word)
            self._words[product_id] = tuple(seen)
        owners: dict[str, int] = {}
        for product_words in self._words.values():
            for word in product_words:
                owners[word] = owners.get(word, 0) + 1
        self._distinguishing = {
            product_id: tuple(w for w in product_words if owners[w] == 1)
            for product_id, product_words in self._words.items()
        }

    def _read(self, word: str, read_words: Iterable[str]) -> bool:
        return any(
            difflib.SequenceMatcher(None, word, seen).ratio() >= self.min_ratio
            for seen in read_words
        )

    def names(self, read_lines: Sequence[str]) -> OcrAnswer:
        """Products named by one crop's OCR lines."""
        read_words = [w for line in read_lines for w in words(line)]
        named: list[tuple[str, tuple[str, ...]]] = []
        for product_id, product_words in self._words.items():
            distinguishing = self._distinguishing[product_id]
            if not distinguishing:
                continue  # a product that shares every word with another can never be named
            matched = tuple(w for w in product_words if self._read(w, read_words))
            if all(w in matched for w in distinguishing) and len(matched) >= 2:
                named.append((product_id, matched))
        candidates = tuple(g for g, _m in named)
        if len(named) == 1:
            return OcrAnswer(named[0][0], named[0][1], candidates, len(read_words))
        return OcrAnswer(None, (), candidates, len(read_words))

    def combine(self, answers: Sequence[OcrAnswer]) -> OcrAnswer:
        """Merge the answers of several crops of one object (disagreement is a conflict)."""
        candidates: list[str] = []
        matched: tuple[str, ...] = ()
        for answer in answers:
            for product_id in answer.candidates:
                if product_id not in candidates:
                    candidates.append(product_id)
            if answer.product_id is not None and not matched:
                matched = answer.matched
        total = sum(a.raw_words for a in answers)
        if len(candidates) == 1:
            return OcrAnswer(candidates[0], matched, tuple(candidates), total)
        return OcrAnswer(None, (), tuple(candidates), total)
