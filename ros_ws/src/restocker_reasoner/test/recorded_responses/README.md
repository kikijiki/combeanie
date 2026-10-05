# Recorded reasoner responses

Verbatim answers a real backend gave to the failure questions in `test/recorded_failure_cases.hpp`.
`test_recorded_reasoner_responses` replays them through the validator the live client uses, so the
contract is pinned by real model output. They run without a model service.

## The failures

Ten standalone runs of the manipulation acceptance test produced six passes and four motion
failures. The `observed_detail` in each question is the terminal detail the coordinator reported
for one of them. The recovery authorisation permits a bounded replan for all three, which is why a
reasoner is asked about them. A failure the authorisation refuses leaves stopping as the only
permitted primitive, so no question is asked.

## Backend

| | |
| --- | --- |
| Server | `llama-server` (llama.cpp build `b10621-c1d0e7a`), OpenAI-compatible, `http://127.0.0.1:8080` |
| Model | `qwen3.6-heretic` (Qwen3.6 35B-A3B, UD-Q6_K_XL GGUF) |
| Sampling | `temperature 0.0`, `stream false`, `chat_template_kwargs.enable_thinking false` |
| Captured | 2026-09-08 |

## How to capture again

```bash
just build-package restocker_reasoner
./ros_ws/build/restocker_reasoner/reasoner_probe --list
./ros_ws/build/restocker_reasoner/reasoner_probe \
  --case linear_path_truncated --model <model> --timeout-ms 120000
```

The probe prints the query, the answer, and the validator's verdict. The text between
`response_verbatim_begin` and `response_verbatim_end` is what these files hold.

## Files

| File | The failure | Result |
| --- | --- | --- |
| `linear_path_truncated.json` | `approach motion planning failed: linear path stopped 41.6667% of the way to the goal after 17 interpolated waypoints` | accepted, 4.7 s; classified `linear_path_truncated`, recommended `resume_at_recovery_state` (the deterministic policy's choice) |
| `trajectory_execution_aborted.json` | `pre-grasp motion execution failed: trajectory execution was rejected or aborted (MoveIt reported CONTROL_FAILED, code -4)` | accepted, 4.8 s; classified `trajectory_execution_aborted`, same recommendation |
| `free_space_planning_failed.json` | `pre-insert motion planning failed: motion planning did not produce a valid solution (MoveIt reported FAILURE)` | accepted, 3.9 s; classified `free_space_planning_failed`, same recommendation |
| `unconstrained_free_form.json` | The pre-insert question with constrained decoding off. The model emitted the members in its own order and the validator still accepted it. Constrained decoding is a convenience, not the trust boundary. | accepted, 3.0 s |
| `truncated_output.txt` | The same question with the output budget cut to 55 tokens, so the backend ran out mid-object. | rejected: `the response is not strict JSON: an object member name must be a quoted string` |

The model classified all three failures correctly and recommended the deterministic primitive every
time. The question already states what the deterministic policy decided, so it had nothing to add.
For the truncated Cartesian approach its explanation is wrong: it says resuming lets the controller
re-evaluate the path from the last known good position, while the policy goes back to the preceding
free-space traverse so a different arm configuration is chosen. Nothing reads the explanation.

Latencies are 3.0 s to 4.9 s on an idle GPU and 5.3 s to 34.3 s under load, against a recovery
command budget of 15 s.
