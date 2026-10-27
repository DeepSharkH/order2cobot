# porttest: the switch / RJ45 insertion task

Plugging the gripped RJ45 plug into the TP-Link LS108G (skill `.claude/skills/rj45-insert`).
Separate from breadboard insertion (`breadboard/`). Shared robot code lives in `urcore/`.

| File | Role |
|---|---|
| `rj45_b.py`, `cad_run.py` | runners (Option B fast path; CAD-guided run) |
| `skill_b.py` | one-upload socket dispatcher of the RJ45 primitives |
| `urexec.py` | `URExec` with the switch defaults (app `rj45_qa`, client `rj45-skill`, `configs/switch/grip_rj45.yaml`) on top of `urcore.urexec` |
| `urlib.py` | RJ45-only URScript primitives; `LIB` = shared + RJ45 |
| `survey2.py`, `port_survey.py`, `ports.py` | port survey with the gripper camera (`data/ports.yaml`) |
| `locate.py`, `view_pose.py`, `go_view.py` | gripper-camera geometry and view poses (`go_view.W3_REF`: cable reference) |
| `tag_test.py` | grip check from the AprilTag |
| `hande_app.py`, `grip_app_import.py` | Hand-E partial-stroke attempts (failed; kept for reference) |
| `LEARNINGS.md` | the switch notebook (+ bridge / calibration / protoboard notes) |
| `runlog.py`, `telemetry.py`, `plot_telemetry.py`, `record.py`, `urscript_lint.py`, `session_trace/session_trace_to_drawio.py` | moved to `urcore/`; these keep `python -m porttest.<name>` working |

Configs: `configs/switch/` (`grip_rj45.yaml`, `ethernet_exp.yaml`), `configs/products/ls108g.yaml`
(moves to `configs/switch/` once the in-flight cadguide work is committed). Old experiments
(find_port*, qa_run, hand-eye and depth tools, ...): `archive/porttest_legacy/`.
