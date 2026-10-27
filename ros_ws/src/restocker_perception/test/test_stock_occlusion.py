# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Lane stock benchmark: the depth occluder of the robustness campaign (CMB-116)."""

import numpy as np
import pytest
from restocker_perception.stock import benchmark as bm


def _depth():
    return np.full((72, 128), 1.2)


def test_a_near_strip_replaces_the_depth_of_the_asked_share_at_one_edge():
    out, (side, first, last) = bm.occlude_depth(_depth(), "frame-a", 0.25, "near")
    assert last - first == 32 and (first == 0 or last == 128) and side in ("left", "right")
    assert (out[:, first:last] == bm.OCCLUDER_RANGE_M).all()
    assert (np.delete(out, range(first, last), axis=1) == 1.2).all()


def test_a_hole_strip_returns_nothing_and_the_input_is_untouched():
    depth = _depth()
    out, (_side, first, last) = bm.occlude_depth(depth, "frame-b", 0.5, "hole")
    assert (out[:, first:last] == 0.0).all() and (depth == 1.2).all()


def test_the_edge_is_a_function_of_the_frame_id_only_and_both_edges_occur():
    sides = {bm.occlude_depth(_depth(), f"f{i}", 0.2, "near")[1][0] for i in range(40)}
    assert sides == {"left", "right"}
    assert (
        bm.occlude_depth(_depth(), "same", 0.2, "near")[1]
        == bm.occlude_depth(_depth(), "same", 0.2, "near")[1]
    )


def test_an_unknown_kind_is_refused():
    with pytest.raises(ValueError, match="near"):
        bm.occlude_depth(_depth(), "f", 0.2, "smoke")
