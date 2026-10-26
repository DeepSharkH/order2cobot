# Simulation

A simulated UR3e + Hand-E behind a simulated **UR AI Agent Bridge**. The skills reach it exactly
like the robot, through the same MCP endpoint path. Only the URL changes:

```bash
python -m simulation                     # :8095, real time; --rate 0 = as fast as possible
export UR_BRIDGE_URL=http://127.0.0.1:8095/universal-robots/ai-agent-bridge/mcp/v1/
python -m porttest.urexec 'textmsg("hello")'          # or any skill step, SkillB, move_checked
```

Watch it in the dashboard's **Simulation** tab: the arm and the objects in 3D, the TCP path and
contact force, the URScript that runs (executing line highlighted), every bridge call and socket
command sent to the arm, and the controller log.

## What is simulated

| Robot side | Simulator |
|---|---|
| AI Agent Bridge (MCP, 101 tools) | same path, tool schemas and response shapes (captured read-only from the robot, `reference/bridge_capture.json`); program/robot/log tools implemented, the rest answer `not_simulated` |
| PolyScope X apps | same app list and ids (rj45_qa = 10), `ur-script` nodes, load/play/pause/stop, persisted in `data/sim/programs.json` |
| Controller log (`psx_logs`) | `PROGRAM_XXX_STARTED/STOPPED`, `text_msg`, runtime errors, protective stops |
| URScript | compiled to Python (`urscript.py`); the urlib primitives and the skill_b dispatcher run unchanged, including the sockets (skill_b connects back to 192.168.2.250:50101, the Robotiq socket on 63352 is simulated) |
| Motion | controller DH table, movej/movel trapezoids, speedl/stopl, force_mode admittance, IK, at 125 Hz in simulated time |
| F/T sensor, contact | spring contact on the TCP point against the plate, the LS108G (8 ports with clearance, mouth funnel, seat depth, holding friction) and the breadboard holes; protective stop above 100 N |
| Safety / modes | Remote/Local, tier (`--tier`), protective stop + unlock, measured bridge delays (`--latency 1`) |

The objects sit at the cadguide pose (`data/cad/*.yaml`) or the snapshot in `world.yaml`.
`--truth-offset-mm dx dy dz` moves them away from that pose, so the searches have to find them.
Contact constants (stiffness, friction, port holding force) are in `world.yaml`.

**Not simulated:** cameras (the dashboard keeps showing the real ones), RTDE telemetry
(`porttest.telemetry` reads the robot), the switch link check, gripper grasping, collisions of
the arm itself, threads in URScript.

## Files

`__main__.py` CLI · `bridge.py` MCP server + `/sim/*` viewer API · `controller.py` apps, play,
log · `urscript.py` compiler + built-ins · `robot.py` kinematics, motion, force mode, F/T,
Hand-E · `world.py` + `world.yaml` contact world · `dashboard.py` + `web/` the dashboard tab
(routes added in `dashboard/server.py`) · tests in `tests/simulation/`.
