# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Seeded scenario benchmarking for the restocking system."""

# ``runner`` needs a ROS environment; ``scenarios``, ``taxonomy``, ``report``, ``plots`` and
# ``transfer_reliability`` do not, so a stored campaign can be re-aggregated and re-plotted
# without rclpy. Nothing is imported here so that importing one of those never pulls in rclpy.

# 2: provenance.arm (UR10e population gate) and provenance.hardware.load_average_at_finish,
# added for the predeclared paired transfer-reliability campaign. Field additions only; 1-era
# records still load and are reported as arm-ineligible when they cannot name the arm.
#
# 3: provenance.obstacle_perception — the effective depth-obstacle pipeline value each run
# launched with (explicit argument or shipped baseline default), added with Card 023 whose
# default flip would otherwise be invisible in stored campaigns. Field addition only; older
# records simply lack it.
RECORD_SCHEMA_VERSION = 3
