# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Aggregate a benchmark campaign into a summary, a report and plots."""
# Reports the pass rate and the distinct failure modes with their frequencies.
# Needs no ROS environment, so a stored campaign stays readable after its workspace is gone.

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import statistics

from restocker_benchmarks import plots, recovery, taxonomy

# Expected benchmark metrics, each with how this harness gets it or why it does not.
METRIC_COVERAGE: tuple[tuple[str, str, str], ...] = (
    (
        "Object-detection success rate",
        "measured (see caveat)",
        "Fraction of stocked products present in the world-state snapshot before the first goal. "
        "The perception backend is the Gazebo ground-truth adapter, so this measures ingestion "
        "and admission, not a detector.",
    ),
    (
        "6-DoF pose translation error",
        "measured (see caveat)",
        "Distance between each observed product pose and the pose the scenario file fixed. With "
        "ground-truth perception this is the settling error of the physics, not a pose "
        "estimator's error.",
    ),
    (
        "6-DoF pose rotation error",
        "not measured",
        "Every catalog product is a cylinder spawned upright with zero yaw, so a rotation error "
        "against the scenario file would be a measurement of nothing.",
    ),
    (
        "Object-tracking continuity",
        "not measured",
        "Would need the snapshot sampled throughout a run rather than at its boundaries. The "
        "world state already tests stable identity across repeated observations.",
    ),
    (
        "Planning success rate",
        "measured indirectly",
        "Goals whose terminal family is a planning family, over all goals. The coordinator plans "
        "six segments per transfer and does not report them individually.",
    ),
    (
        "Planning latency",
        "NOT SEPARABLE",
        "Reported only as combined plan-and-execute time per goal. A motion port plans and "
        "executes a segment in one call, so the PLAN_* feedback state holds for both and the "
        "paired EXECUTE_* state holds for effectively zero. Splitting them by state gave "
        "'execution 0.0 s' on every successful goal, which is the state machine's shape and not "
        "a measurement. A real planning latency needs instrumentation inside MoveItMotionPort.",
    ),
    (
        "Trajectory execution success rate",
        "measured indirectly",
        "Goals reaching each EXECUTE_* state, from the feedback trace. The state is entered after "
        "the segment ran, so reaching it is the evidence that execution succeeded.",
    ),
    (
        "Grasp success rate",
        "measured",
        "Goals whose feedback trace reached ATTACH_TRANSACTION or beyond.",
    ),
    (
        "Placement success rate",
        "measured",
        "Goals whose feedback trace reached VERIFY_PLACEMENT or beyond.",
    ),
    (
        "Recovery-selection accuracy",
        "not measured",
        "Requires a known-correct recovery per failure to compare against. No such ground truth "
        "exists yet.",
    ),
    (
        "Recovery success rate",
        "measured (see caveat)",
        "Goals that entered a recovery attempt and still succeeded, over goals that entered one. "
        "A single rate over a shared, depleting budget: the attempts behind it are measurably "
        "not independent, so it must not be raised to a power to project a residual. See "
        "the recovery-independence section of the report for the conditional rates.",
    ),
    ("End-to-end task cycle time", "measured", "Wall and simulation time per goal."),
    (
        "Minimum observed clearance",
        "NOT AVAILABLE",
        "RestockProduct.Result.minimum_clearance_m is copied from a metrics struct that no "
        "production path assigns, so it is 0.0 on every run including successful ones. Exporting "
        "it would publish a default as a measurement. Real clearance needs a recorded joint "
        "trace put through tools/diagnostics/collide.py.",
    ),
    (
        "Number of retries",
        "measured (see caveat)",
        "Recovery attempts, counted from the feedback trace. Result.retry_count is NOT usable "
        "and is not what this reports: the per-operation counter is reset by every state "
        "transition before the terminal transition reads it, so retry_count is 0 on every goal "
        "in every campaign recorded so far. A per-operation retry leaves no trace entry either, "
        "so the count here is recoveries only and is a lower bound on submissions.",
    ),
    (
        "Number of operator escalations",
        "measured",
        "Goals terminating with STATUS_OPERATOR_REQUIRED.",
    ),
)

CLEARANCE_UNAVAILABLE_REASON = next(
    how for metric, _status, how in METRIC_COVERAGE if metric == "Minimum observed clearance"
)

# Stages a complete transfer passes, for the reach table.
REPORTED_STAGES: tuple[tuple[str, int], ...] = (
    ("selected a pair", 2),
    ("reserved the task", 3),
    ("planned the pre-grasp", 5),
    ("executed the pre-grasp", 6),
    ("executed the approach", 8),
    ("closed the gripper", 9),
    ("verified the grasp", 10),
    ("attached the product", 11),
    ("executed the retract", 13),
    ("generated a placement", 15),
    ("executed the pre-insert", 17),
    ("executed the insert", 19),
    ("detached the product", 21),
    ("verified the placement", 22),
    ("executed the retreat", 24),
    ("updated the inventory", 25),
)

_STAGE_RANK = {state: rank for rank, state in enumerate(taxonomy.NOMINAL_STATE_ORDER)}


def load_runs(campaign: Path) -> list[dict]:
    """Load every run record under ``campaign``, newest last."""
    runs = []
    for path in sorted(campaign.rglob("run.json")):
        try:
            runs.append(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, json.JSONDecodeError) as error:
            print(f"warning: skipping unreadable record {path}: {error}")
    return runs


def _reached(goal: dict, state: int) -> bool:
    """Whether a goal's feedback trace reached ``state`` on the nominal forward path."""
    target = _STAGE_RANK.get(state)
    if target is None:
        return False
    return any(
        _STAGE_RANK.get(int(entry["state"]), -1) >= target for entry in goal.get("trace", [])
    )


def _goals(runs: list[dict]) -> list[tuple[dict, dict]]:
    return [(run, goal) for run in runs for goal in run.get("goals", [])]


def classify_goal(goal: dict) -> tuple[str, str]:
    """Return the family and signature for a goal, recomputed from what the system reported."""
    # Recomputed, not read from the record, so taxonomy additions re-label older campaigns.
    outcome = goal["outcome"]
    status = int(outcome["status"])
    if status == taxonomy.HARNESS_STATUS:
        # Harness outcomes group on their short reason.
        reason = outcome.get("reason") or outcome.get("detail") or ""
        return taxonomy.classify_family(status, reason), taxonomy.signature(status, reason)
    detail = outcome.get("detail_full") or outcome.get("detail") or ""
    cause = taxonomy.cause_detail(goal.get("trace", []))
    family = taxonomy.classify_family(status, cause or detail)
    return family, taxonomy.signature(status, detail, cause)


def summarize(runs: list[dict]) -> dict:
    """Reduce a campaign to the numbers and the failure taxonomy the report is built from."""
    pairs = _goals(runs)
    succeeded = [
        goal for _, goal in pairs if goal["outcome"]["status"] == taxonomy.STATUS_SUCCEEDED
    ]
    failed = [
        (run, goal)
        for run, goal in pairs
        if goal["outcome"]["status"] != taxonomy.STATUS_SUCCEEDED
    ]

    signatures: dict[str, dict] = {}
    for run, goal in failed:
        outcome = goal["outcome"]
        family, signature = classify_goal(goal)
        entry = signatures.setdefault(
            signature,
            {
                "signature": signature,
                "family": family,
                "count": 0,
                "independent": 0,
                "cascaded": 0,
                "scenarios": Counter(),
                "furthest_states": Counter(),
                "example_detail": outcome.get("detail_full") or outcome["detail"],
                # The result detail is sometimes terse; the cause sentence is kept separately.
                "example_cause_detail": taxonomy.cause_detail(goal.get("trace", [])),
                "example_run": run.get("run_id"),
            },
        )
        entry["count"] += 1
        if goal.get("preceded_by_failure"):
            entry["cascaded"] += 1
        else:
            entry["independent"] += 1
        entry["scenarios"][run["scenario"]] += 1
        entry["furthest_states"][goal.get("furthest_state_name", "none")] += 1

    for entry in signatures.values():
        entry["scenarios"] = dict(entry["scenarios"])
        entry["furthest_states"] = dict(entry["furthest_states"])

    stage_reach = [
        {
            "stage": name,
            "state": taxonomy.state_name(state),
            "reached": sum(1 for _, goal in pairs if _reached(goal, state)),
        }
        for name, state in REPORTED_STAGES
    ]

    recovered = [goal for _, goal in pairs if goal.get("recovery_attempts", 0) > 0]
    wall_times = [goal["wall_s"] for goal in succeeded if "wall_s" in goal]
    sim_times = [goal["elapsed_sim_s"] for goal in succeeded if "elapsed_sim_s" in goal]

    observation = [
        run["ground_truth_observation"] for run in runs if "ground_truth_observation" in run
    ]
    stocked = sum(entry["stocked"] for entry in observation)
    observed = sum(entry["observed"] for entry in observation)
    settling = [
        entry["max_translation_error_m"]
        for entry in observation
        if entry.get("max_translation_error_m") is not None
    ]

    return {
        "runs": len(runs),
        "runs_completed": sum(1 for run in runs if run.get("outcome") == "completed"),
        "runs_all_goals_succeeded": sum(
            1
            for run in runs
            if run.get("outcome") == "completed"
            and run.get("goals")
            and all(
                goal["outcome"]["status"] == taxonomy.STATUS_SUCCEEDED for goal in run["goals"]
            )
        ),
        "goals": len(pairs),
        "goals_succeeded": len(succeeded),
        "goals_failed": len(failed),
        "goals_failed_independent": sum(
            1 for _, goal in failed if not goal.get("preceded_by_failure")
        ),
        "failure_signatures": sorted(
            signatures.values(), key=lambda entry: (-entry["count"], entry["signature"])
        ),
        "families": dict(Counter(classify_goal(goal)[0] for _, goal in pairs).most_common()),
        "stage_reach": stage_reach,
        "operator_escalations": sum(
            1 for _, goal in pairs if goal["outcome"]["status_name"] == "OPERATOR_REQUIRED"
        ),
        "retry_count_total": sum(goal.get("retry_count", 0) for _, goal in pairs),
        "recovery_goals": len(recovered),
        "recovery_goals_succeeded": sum(
            1 for goal in recovered if goal["outcome"]["status"] == taxonomy.STATUS_SUCCEEDED
        ),
        # Whether recovery attempts behave as independent redraws.
        "recovery_independence": recovery.summarize(runs),
        "cycle_time_wall_s": _distribution(wall_times),
        "cycle_time_sim_s": _distribution(sim_times),
        "motion_sim_s": _distribution(
            [
                taxonomy.phase_durations(
                    goal.get("trace", []), float(goal.get("elapsed_sim_s", 0.0))
                )["motion_sim_s"]
                for goal in succeeded
            ]
        ),
        "non_motion_sim_s": _distribution(
            [
                taxonomy.phase_durations(
                    goal.get("trace", []), float(goal.get("elapsed_sim_s", 0.0))
                )["other_sim_s"]
                for goal in succeeded
            ]
        ),
        "ground_truth_observation": {
            "products_stocked": stocked,
            "products_observed": observed,
            "max_settling_translation_error_m": max(settling) if settling else None,
        },
        "minimum_clearance_m": None,
        "minimum_clearance_unavailable_reason": CLEARANCE_UNAVAILABLE_REASON,
    }


def _distribution(values: list[float]) -> dict[str, float] | None:
    if not values:
        return None
    return {
        "count": len(values),
        "min": min(values),
        "median": statistics.median(values),
        "max": max(values),
        "mean": statistics.fmean(values),
    }


def _rate(numerator: int, denominator: int) -> str:
    if denominator == 0:
        return "n/a"
    return f"{numerator}/{denominator} ({numerator / denominator * 100:.0f}%)"


def _by_scenario(runs: list[dict]) -> dict[str, list[dict]]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for run in runs:
        grouped[run["scenario"]].append(run)
    return dict(sorted(grouped.items()))


def _render_recovery_independence(summary: dict) -> list[str]:
    """Render the evidence for or against treating recovery attempts as independent redraws."""
    evidence = summary.get("recovery_independence") or {}
    rows = evidence.get("independence") or []
    sharing = evidence.get("budget_sharing") or {}
    lines = [
        "",
        "## Are recovery attempts independent?",
        "",
        "Any reliability estimate that multiplies a per-attempt failure rate over the recovery "
        "budget assumes two things, and this section measures both rather than assuming them. "
        "Read it before quoting a residual: a product over correlated attempts understates the "
        "joint failure probability, and understates it by more the larger the budget is.",
        "",
    ]
    if not rows:
        lines += [
            "No goal in this campaign entered recovery, so neither assumption is tested here. "
            "That is not evidence they hold.",
            "",
        ]
        return lines

    lines += [
        "**Independence.** Of the goals whose first recovery was spent on a given cause, how "
        "many met the *same* cause again on a later recovery slot. If each attempt were a fresh "
        "draw, this would sit at that cause's unconditional rate; measurably above it means a "
        "failed attempt predicts the next one.",
        "",
        "| First cause to spend a recovery | Recurred later | Rate | 95% CI (Wilson) |",
        "| --- | ---: | ---: | ---: |",
    ]
    for row in rows:
        label = "**all causes pooled**" if row["family"] == "ALL" else f"`{row['family']}`"
        lines.append(
            f"| {label} | {row['recurred']}/{row['entered']} | {row['rate'] * 100:.0f}% | "
            f"{recovery.format_interval(row['interval'])} |"
        )

    budget = sharing.get("budget")
    entering = sharing.get("goals_entering_recovery", 0)
    lines += [
        "",
        f"**Budget sharing.** `task.max_recovery_attempts` is {budget}, and it is spent across "
        "the whole `RestockProduct` goal rather than per operation: the counter is never reset "
        "between phases, and not on a successful one either. So a fault arriving late can find "
        "the pool already emptied by an unrelated fault, and gets fewer attempts than a "
        "per-fault estimate would give it.",
        "",
        f"- Goals that entered recovery at all: **{entering}**",
        f"- Of those, goals that died with the pool exhausted: "
        f"**{_rate(sharing.get('died_with_the_pool_exhausted', 0), entering)}**",
        f"- Of those, goals that died after a *different* cause had already taken a slot: "
        f"**{_rate(sharing.get('died_after_another_fault_spent_a_slot', 0), entering)}**",
        "",
        "Each cause is counted once per goal, at its first appearance, because what is measured "
        "is the state of the pool when it arrived. A `0` means it had the full budget.",
        "",
        "| Cause | Goals | Slots already spent when it first appeared | Arrived with the pool "
        "empty |",
        "| --- | ---: | --- | ---: |",
    ]
    for entry in sharing.get("families", []):
        spread = ", ".join(
            f"{slot} spent: {count}" for slot, count in entry["first_seen_at_slot"].items()
        )
        lines.append(
            f"| `{entry['family']}` | {entry['goals']} | {spread} | "
            f"{entry['arrived_with_pool_empty']} |"
        )
    lines += [
        "",
        "A per-operation retry is deliberately absent from both tables. "
        "`task.max_operation_retries` re-submits an operation without a state transition, so it "
        "leaves no trace entry and `Result.retry_count` reads 0 on every goal ever recorded. For "
        "a Cartesian segment that repeat also carries essentially no new information: it runs "
        "the same interpolation against the same retained pose target from an arm that a "
        "planning failure never moved. Only a recovery re-runs the traverse that chose the "
        "configuration, so only a recovery is counted as an attempt here, and the attempt counts "
        "above are a lower bound on submissions.",
        "",
    ]
    return lines


def render_markdown(runs: list[dict], summary: dict, plot_names: list[str]) -> str:
    """Render the campaign report."""
    lines: list[str] = ["# Benchmark campaign", ""]
    if not runs:
        lines.append("No runs were found. Run `just benchmark <scenario>` first.")
        return "\n".join(lines) + "\n"

    commits = sorted({run["provenance"].get("git_describe") or "unknown" for run in runs})
    seeds = sorted({str(run.get("seed")) for run in runs})
    started = sorted(run.get("started_utc", "") for run in runs)
    hardware = runs[-1]["provenance"]["hardware"]
    lines += [
        f"- Runs: **{summary['runs']}** between `{started[0]}` and `{started[-1]}` (UTC)",
        f"- Commit: {', '.join(f'`{commit}`' for commit in commits)}",
        f"- Flake lock: `{runs[-1]['provenance'].get('flake_lock_sha256', 'unknown')[:16]}`",
        f"- Seeds: {', '.join(f'`{seed}`' for seed in seeds)}",
        f"- Planner: `{runs[-1]['provenance']['planner_backend'].get('planning_plugins')}`, "
        f"perception `{runs[-1]['provenance'].get('perception_backend')}`, "
        f"model backend `{runs[-1]['provenance']['model_backend']['name']}`",
        f"- Hardware: {hardware.get('cpu_model')} "
        f"({hardware.get('cpu_count')} threads), {hardware.get('platform')}",
        "",
        "## Headline",
        "",
        f"- Goals succeeded: **{_rate(summary['goals_succeeded'], summary['goals'])}**",
        f"- Runs where every goal succeeded: "
        f"**{_rate(summary['runs_all_goals_succeeded'], summary['runs'])}**",
        f"- Distinct failure causes: **{len(summary['failure_signatures'])}**",
        f"- Operator escalations: {summary['operator_escalations']}",
        "",
        "A run is a full headless launch of Gazebo, the controllers, MoveIt and the coordinator, "
        "driving one scenario's goal sequence. A goal that fails does not stop its run, so a "
        "later goal in the same run is marked *cascaded* and counted separately: it inherited a "
        "robot that may still be holding a product.",
        "",
        "## Per scenario",
        "",
        "| Scenario | Runs | Goals | Goals succeeded | Clean runs |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for name, scenario_runs in _by_scenario(runs).items():
        scoped = summarize(scenario_runs)
        lines.append(
            f"| `{name}` | {scoped['runs']} | {scoped['goals']} | "
            f"{_rate(scoped['goals_succeeded'], scoped['goals'])} | "
            f"{_rate(scoped['runs_all_goals_succeeded'], scoped['runs'])} |"
        )

    lines += ["", "## Failure taxonomy", ""]
    if not summary["failure_signatures"]:
        lines += [
            "No goal failed in this campaign. That is a result about this campaign's size, not "
            "about the system: the acceptance scenarios here have been measured failing before, "
            "so a campaign this small showing none is consistent with a pass rate well under 1.",
            "",
        ]
    else:
        lines += [
            "Grouped by the coordinator's terminal `detail` with every number masked, so two "
            "goals share a row only when the system said the same thing about them. Where the "
            "goal entered a fault state, the detail it published there is appended after `<-`: "
            "the terminal result is sometimes the terse one, and two failures that both end "
            '"motion inhibited; operator required" for different reasons must not share a row. '
            "*Depth* is the furthest state on the nominal forward path the feedback trace "
            "reached.",
            "",
            "| Cause | Family | Count | Independent | Cascaded | Depth |",
            "| --- | --- | ---: | ---: | ---: | --- |",
        ]
        for entry in summary["failure_signatures"]:
            depth = ", ".join(
                f"{state} x{count}" for state, count in sorted(entry["furthest_states"].items())
            )
            signature = entry["signature"]
            if len(signature) > 170:
                # Elide the middle: causes often differ only at the end.
                signature = f"{signature[:105]} [...] {signature[-55:]}"
            lines.append(
                f"| {signature.replace('|', '/')} | `{entry['family']}` | {entry['count']} | "
                f"{entry['independent']} | {entry['cascaded']} | {depth} |"
            )
        lines += ["", "### Full detail of each cause", ""]
        for entry in summary["failure_signatures"]:
            lines += [
                f"**`{entry['family']}` x{entry['count']}** (example from run "
                f"`{entry['example_run']}`)",
                "",
                "Terminal result detail:",
                "",
                "```text",
                entry["example_detail"],
                "```",
                "",
            ]
            if entry.get("example_cause_detail"):
                lines += [
                    "Fault-state feedback, which is where the cause is named:",
                    "",
                    "```text",
                    entry["example_cause_detail"],
                    "```",
                    "",
                ]

    lines += _render_recovery_independence(summary)

    lines += [
        "## How far goals got",
        "",
        "Cumulative: each row counts every goal whose feedback trace reached that stage, so the "
        "step where the count drops is the step that failed.",
        "",
        "| Stage | Goals reaching it |",
        "| --- | ---: |",
    ]
    for entry in summary["stage_reach"]:
        lines.append(f"| {entry['stage']} (`{entry['state']}`) | {entry['reached']} |")

    lines += [
        "",
        "## Timing",
        "",
        "| Quantity | Count | Min | Median | Max |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for label, key in (
        ("Cycle time, wall (s)", "cycle_time_wall_s"),
        ("Cycle time, simulation (s)", "cycle_time_sim_s"),
        ("Motion segments, plan and execute, simulation (s)", "motion_sim_s"),
        ("Gripper, attachment and verification, simulation (s)", "non_motion_sim_s"),
    ):
        distribution = summary[key]
        if distribution is None:
            lines.append(f"| {label} | 0 | - | - | - |")
        else:
            lines.append(
                f"| {label} | {distribution['count']} | {distribution['min']:.1f} | "
                f"{distribution['median']:.1f} | {distribution['max']:.1f} |"
            )
    lines += [
        "",
        "Successful goals only; a failed goal's duration measures where it stopped. Planning and "
        "execution are one row and not two: a motion port plans and executes a segment in a "
        "single call, so the PLAN_* feedback state holds for the whole operation and the paired "
        "EXECUTE_* state records only that the trajectory ran. Splitting them yielded "
        '"execution 0.0 s" on every successful goal -- an artefact of the state machine\'s '
        "shape, not a measurement.",
    ]

    observation = summary["ground_truth_observation"]
    error = observation["max_settling_translation_error_m"]
    lines += [
        "",
        "## Scenario fidelity",
        "",
        "- Stocked products present in the world state: "
        + _rate(observation["products_observed"], observation["products_stocked"]),
        "- Largest distance between an observed product and its seeded pose: "
        + (f"{error * 1000:.4f} mm" if error is not None else "n/a"),
        "",
        "This is the evidence that a seed reproduces a scenario. It is not a perception metric: "
        "the backend is the Gazebo ground-truth adapter.",
        "",
        "## Metric coverage",
        "",
        "Every metric a benchmark is expected to record, and how this harness gets it.",
        "",
        "| Metric | Status | How |",
        "| --- | --- | --- |",
    ]
    for metric, status, how in METRIC_COVERAGE:
        lines.append(f"| {metric} | {status} | {how} |")

    if plot_names:
        lines += ["", "## Plots", ""]
        lines += [f"- `{name}`" for name in plot_names]

    lines += [
        "",
        "## Reproducing this",
        "",
        "```bash",
        "just build",
        "just benchmark baseline_transfers          # one headless run",
        "just benchmark-all                          # every scenario once",
        "just benchmark-report                       # regenerate this file",
        "```",
        "",
        "Every run archives the scenario document it spawned and the launch console beside its "
        "`run.json`, so a failure here replays from the same seed against the recorded commit.",
        "",
    ]
    return "\n".join(lines) + "\n"


def render_plots(summary: dict, runs: list[dict], directory: Path) -> list[str]:
    """Write the campaign's SVG plots and return their file names."""
    directory.mkdir(parents=True, exist_ok=True)
    written: list[str] = []

    families = summary["families"]
    bars = [
        plots.Bar(family, float(count), str(count), highlight=family != "succeeded")
        for family, count in sorted(families.items(), key=lambda item: (-item[1], item[0]))
    ]
    (directory / "goal-outcomes.svg").write_text(
        plots.horizontal_bars(
            "Goal outcomes by family",
            f"{summary['goals']} goals over {summary['runs']} runs",
            bars,
        ),
        encoding="utf-8",
    )
    written.append("goal-outcomes.svg")

    depth = Counter()
    for run in runs:
        for goal in run.get("goals", []):
            if goal["outcome"]["status"] != taxonomy.STATUS_SUCCEEDED:
                depth[goal.get("furthest_state_name", "none")] += 1
    ordered = sorted(
        depth.items(),
        key=lambda item: _STAGE_RANK.get(
            next(
                (state for state, name in taxonomy.STATE_NAMES.items() if name == item[0]),
                -1,
            ),
            -1,
        ),
    )
    (directory / "failure-depth.svg").write_text(
        plots.horizontal_bars(
            "Where failing goals stopped",
            "furthest state on the nominal forward path, per failed goal",
            [plots.Bar(name, float(count), str(count), highlight=True) for name, count in ordered],
        ),
        encoding="utf-8",
    )
    written.append("failure-depth.svg")

    wall = [
        goal["wall_s"]
        for run in runs
        for goal in run.get("goals", [])
        if goal["outcome"]["status"] == taxonomy.STATUS_SUCCEEDED and "wall_s" in goal
    ]
    (directory / "cycle-time.svg").write_text(
        plots.histogram(
            "Cycle time of a successful transfer",
            "wall-clock seconds per succeeded goal",
            wall,
            "s",
        ),
        encoding="utf-8",
    )
    written.append("cycle-time.svg")
    return written


def main(argv: list[str] | None = None) -> int:
    """Aggregate a campaign directory into summary.json, report.md and plots."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "campaign",
        nargs="?",
        default="benchmark-results",
        help="directory holding run.json records (default: benchmark-results)",
    )
    parser.add_argument("--output", default=None, help="where to write the report")
    arguments = parser.parse_args(argv)

    campaign = Path(arguments.campaign)
    output = Path(arguments.output) if arguments.output else campaign
    if not campaign.is_dir():
        raise SystemExit(f"no such campaign directory: {campaign}")

    runs = load_runs(campaign)
    summary = summarize(runs)
    output.mkdir(parents=True, exist_ok=True)
    plot_names = render_plots(summary, runs, output / "plots") if runs else []
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8"
    )
    (output / "report.md").write_text(
        render_markdown(runs, summary, [f"plots/{name}" for name in plot_names]),
        encoding="utf-8",
    )
    print(
        f"[report] {summary['runs']} run(s), {summary['goals']} goal(s), "
        f"{summary['goals_succeeded']} succeeded, "
        f"{len(summary['failure_signatures'])} distinct failure cause(s) -> {output / 'report.md'}"
    )
    return 0
