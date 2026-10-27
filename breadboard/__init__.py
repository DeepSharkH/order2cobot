"""Breadboard insertion: the robot side of Circuit Assembly (see breadboard/README.md).

Input: the plan Circuit Assembly hands off (data/circuit_designs/<id>/plan.json, the CSP
solver's verified assembly_order). Tool: Jan's custom fingers on the Hand-E
(configs/breadboard/gripper.yaml). Positions: the breadboard CAD hole table placed in base by
the manually taught corner inserts (configs/breadboard/calibration.yaml).

Modules: board (hole id -> board-frame mm), calibration (corner inserts -> board pose in
base, hole -> base), teach (record the manual corner inserts and workspace poses), grasp_calib
(TODO: measure the part in the fingers after every grasp), run_plan (TODO: the executor).
Robot motion goes through urcore only; nothing here imports porttest (the switch task).
"""
