# restocker_reasoner

Optional advisory client. It owns the recommendation contract, the strict reader that
validates it, the HTTP-backed implementation, a deterministic fake, and the audit record. It can
only select among recovery primitives the task executor already implements and already authorises,
and the baseline never requires it.

## What is here

| Header | What it is |
| --- | --- |
| `strict_json.hpp` | A JSON reader for text this process did not write. Accepts exactly RFC 8259 and rejects every convenience a forgiving parser allows: comments, trailing commas, unquoted keys, `NaN`, leading zeros, duplicate members, invalid UTF-8, trailing content. Depth and byte count are bounded. |
| `recovery_advice.hpp` | The closed primitive and failure-class enums, the question, the validated recommendation, and `parse_recovery_advice`, which rejects by default. |
| `recovery_advisor_port.hpp` | The abstract port, and `RecoveryAdviceMailbox`, the one-slot letterbox an advisor's thread writes into. |
| `http_recovery_advisor.hpp` | One implementation, for an OpenAI-compatible chat backend. Owns a worker thread, enforces a hard steady-clock deadline, and opens a circuit breaker after consecutive failures. |
| `recovery_audit_log.hpp` | Append-only JSON Lines recording the decision, not only the outcome. |
| `fake_recovery_advisor.hpp` | The deterministic double. It ships beside the interface because the tests that most need it live in `restocker_task_executor`. |

## The primitives

```text
retry_segment              resubmit the failed segment unchanged (the bounded operation retry)
resume_at_recovery_state   the deterministic replan, resuming where recovery_resume_for() says
abandon_task               stop and hand the cell to an operator
```

There is no fourth. Every member is something the executor already does; naming one is not the
same as being allowed to take it, and the executor decides which are permitted at the instant an
action would run.

## The safety property

A recommendation is followed only when it is (a) permitted by the authorisation recomputed at the
moment of the decision, and (b) no less cautious than what the deterministic policy had already
chosen. So an advisor can decline a recovery and can agree with one. It cannot talk the system
into acting, cannot reach a primitive the authorisation refused, and cannot make anything happen
that would not have happened without it. Absent, slow, malformed, hostile, and late are one
behaviour: no recommendation.

## Seeing what a model says

```bash
just build-package restocker_reasoner
./ros_ws/build/restocker_reasoner/reasoner_probe --list
./ros_ws/build/restocker_reasoner/reasoner_probe --case linear_path_truncated --model <model>
```

`reasoner_probe` asks a running backend one reproduced failure question through the real client and
prints the answer verbatim beside the validator's verdict. It is a test-time diagnostic and is not
installed. `test/recorded_responses/README.md` holds the answers a local llama.cpp backend gave,
with their latencies.
