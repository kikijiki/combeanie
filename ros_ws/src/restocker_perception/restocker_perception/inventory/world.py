# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The world state's snapshot as the inventory's WorldView."""
# Duck-typed on the field names of restocker_interfaces/msg/WorldStateSnapshot, so it runs on a
# message or on a stand-in without importing ROS.

from __future__ import annotations

from typing import Any

from restocker_perception.inventory.model import WorldLane, WorldTrayItem, WorldView

TRACKING_TRACKED = 0
TRACKING_OCCLUDED = 1
GRASP_FREE = 0


def world_view_from_snapshot(snapshot: Any) -> WorldView:
    """Lanes with their ledger SKUs, and the tray: tracked, free objects no lane accounts for.

    A lane accounts for an object that is in its ledger (``contents``) or that its fresh
    occupancy evidence names (``observed_source_object_ids``); everything else the world state
    tracks and nobody holds is on the tray. In simulation the world state is fed ground truth by
    default, so this is the simulator's belief and not a sensor reading.
    """
    objects = {int(o.id): o for o in snapshot.objects}
    accounted_ids: set[int] = set()
    accounted_sources: set[str] = set()
    lanes: dict[str, WorldLane] = {}
    for lane in snapshot.lanes:
        contents = [int(i) for i in lane.contents]
        accounted_ids.update(contents)
        accounted_sources.update(str(s) for s in lane.observed_source_object_ids)
        skus = tuple(
            (objects[i].sku if objects[i].has_sku else None) if i in objects else None
            for i in contents
        )
        lanes[str(lane.id)] = WorldLane(
            lane_id=str(lane.id),
            expected_sku=str(lane.expected_sku) if lane.has_expected_sku else None,
            skus=skus,
            ledger_unreliable=bool(lane.ledger_unreliable),
        )
    tray = tuple(
        WorldTrayItem(sku=str(o.sku) if o.has_sku else None, object_id=str(o.source_object_id))
        for o in sorted(objects.values(), key=lambda o: int(o.id))
        if int(o.id) not in accounted_ids
        and str(o.source_object_id) not in accounted_sources
        and int(o.tracking_state) in (TRACKING_TRACKED, TRACKING_OCCLUDED)
        and int(o.grasp_state) == GRASP_FREE
    )
    return WorldView(revision=int(snapshot.revision), lanes=lanes, tray=tray)
