# Archive

Code no current workflow uses, kept for reference (2026-10-27 cleanup). Nothing here is
packaged or collected by pytest, and the imports inside still point at the old locations, so
it does not run as is.

- `porttest_legacy/`: superseded switch experiments (find_port*, qa_run, rj45_skill,
  plan_views, approach_shots, touch_board), the 10-24/25 hand-eye and depth tools (handeye_*,
  joint_solve, fcam_calib, look, depth_*, square_from_depth, snap, dash), the old touch
  geometry (geometry.py; `urpose`/`urvec` moved to urcore.urexec), the URScript primitives only
  they used (urlib_legacy.py), and their calibration outputs (calibration/: handeye_gripper,
  result.yaml; replaced by configs/calibration/scene.yaml).
- `simple_agent/`: the LLM .urpx program writer (agents/simple, agents/llm, agents/robot
  async bridge client, scripts/generate_urpx.py, configs/llm, configs/robot.yaml, tests) and
  scripts/wire_from_cloud.py (challenge2 program flow; its plane vision moved to
  agents/vision/plane.py). Every live robot path uses urcore.urexec over dashboard/ur_bridge.py.
