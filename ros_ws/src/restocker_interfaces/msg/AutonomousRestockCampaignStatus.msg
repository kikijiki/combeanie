# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

# Status emitted by the autonomous campaign. Measurements come from the authoritative
# world-state snapshot after the commanded shelf/tray sweep, after each transfer, and after a
# terminal no-pair result.
uint8 PHASE_WAITING_FOR_COORDINATOR=0
uint8 PHASE_SURVEYING_FRONT=1
uint8 PHASE_SURVEYING_BACK=2
uint8 PHASE_MEASURED=3
uint8 PHASE_RESTOCKING=4
uint8 PHASE_FRONT_FULL=5
uint8 PHASE_STOCK_EXHAUSTED=6
uint8 PHASE_BLOCKED=7
uint8 PHASE_COMPLETE=8
# Card 086 stage 1 (CMB-SPEC-13): the campaign restarted and has not been acknowledged by an
# operator. In-memory unresolved-motion state does not survive a restart, so the previous
# incarnation's motion outcome is unknown; no goal is sent until the acknowledgment.
uint8 PHASE_RECOVERING=9

# The five-mode loop position at emission (Milestone 10 section 1). The phases above report
# campaign progress; the mode is the control structure the campaign is standing in.
uint8 MODE_IDLE=0
uint8 MODE_SURVEY_SHELF=1
uint8 MODE_SURVEY_TRAY=2
uint8 MODE_CONFIRM=3
uint8 MODE_TRANSFER=4

uint8 PRODUCT_CLASS_CAN=1
uint8 PRODUCT_CLASS_SMALL_BOTTLE=2
uint8 PRODUCT_CLASS_LARGE_BOTTLE=3

std_msgs/Header header
uint64 sequence
int64 cycle
uint8 phase
bool front_survey_complete
bool back_survey_complete
uint64 world_revision
# Parallel, stable-order arrays. One entry is published for every known product class.
uint8[] product_classes
uint32[] front_stock
uint32[] front_deficits
uint32[] back_stock
uint32 total_front_stock
uint32 total_front_deficit
uint32 total_back_stock
uint64 successful_transfers
string detail
# Mode-loop position of the publisher when this status was emitted.
uint8 mode
# Cumulative arm-action time for this run, in seconds: shelf lane surveys and coordinated tray
# survey legs on the survey side, RestockProduct transfers on the transfer side. Reported
# separately so a run can say how much arm time went to looking against how much went to moving
# product (Milestone 10 Stage 7).
float64 survey_arm_time_sec
float64 transfer_arm_time_sec
# Card 086 stage 1 (CMB-SPEC-13): true while the campaign holds at least one goal attempt whose
# motion outcome is unproven (an admission or result that timed out, a canceled goal with no
# delivered terminal, a delivered terminal without non-start or stop evidence), and while a
# restart awaits its operator acknowledgment. While true the campaign sends no goal and never
# reports PHASE_COMPLETE; the phase is PHASE_BLOCKED (or PHASE_RECOVERING after a restart).
# Only exact-identity settlement evidence clears it; elapsed time, fresh observations and a
# front-full transition never do. Scope: this campaign process only; a client that calls a
# public endpoint directly is not fenced by it.
bool motion_unresolved
# True from process start until the operator acknowledges the restart.
bool restart_recovery_pending
# One entry per retained unresolved attempt: endpoint, send generation, condition and the exact
# goal UUID once the goal is known.
string[] unresolved_attempts
