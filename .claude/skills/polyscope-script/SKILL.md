---
name: polyscope-script
description: Pitfalls for writing URScript that runs on our UR3e (PolyScope X 10.14, played through the AI Agent Bridge as a ur-script node) - missing built-ins, scoping, loops that starve the controller, force_mode arguments and limits, F/T zeroing and payload, set_tcp persistence, IK guards, the skill_b socket protocol. Use before writing or changing any URScript (urcore/urlib.py, porttest/urlib.py, skill_b dispatcher, breadboard/*.py program generators, a one-off step script) and when a step fails with compile_error_*, invalid_argument_value, force_mode_* aborts, "Lost communication with Controller" or a protective stop.
---

# PolyScope X script writing: pitfalls

The common mistakes that break our URScript steps, taken from the PolyScope X Script Directory
(`~/Downloads/script_directory_PolyScopeX.pdf`) and our failures (`porttest/LEARNINGS.md`).
**[seen]** = it happened to us. **[doc]** = from the manual, not hit yet.

## Before every send

1. `python -m porttest.urscript_lint` after changing `urlib.LIB`. `URExec.run` lints every
   generated script and refuses one with problems. The allowlist (`urcore/urscript_builtins.txt`)
   is the 10.14 function list. If a name is missing from it, it is not on the controller.
2. Units are metres, N, rad, s. Log in mm.
3. Put a whole sequence in one program. A bridge round trip takes ~20 s, a node edit ~9 s.
4. End the script with the flange TCP restored, not while pressing (see TCP and Force mode below).

## Language

- **Python built-ins that don't exist** [seen: `abs` → `compile_error_name_not_found`]:
  `abs`/`fabs` → `norm(x)` (works on int, float, list and pose), `min`/`max` → `if`,
  `len` → `length(x)` / `size(x)` / `x.length()`, `round` → `floor(x + 0.5)`, `str` → `to_str`,
  `int`/`float` → `to_num`/`floor`, `**` → `pow`, `print` → `textmsg`. There is no `for`, `range`,
  `try`, `with`, `lambda`, `is` or list comprehension.
- **`elif` is valid** (if / elif / else / end). Boolean operators are `and`, `or`, `not`, `xor`.
- **A function that returns nothing**: the manual requires `return None`, but our lint flags
  `None`. Return a bool (`return False`) for early exits instead.
- **Types are fixed by the first assignment** [doc]. A list's element type and its **capacity**
  never change: `l = [1, 2]` and later `l = [1, 2, 3]` fails (capacity 2). Size lists for their
  largest use with `make_list(length, init, capacity)`.
- **Arguments are passed by value, lists included** [doc]. A function that changes `n[0]` does
  not change the caller's list. Return the new value instead.
- **Scoping trap** [doc]: variables at the top level of the program are **global**, and a
  variable assigned inside a function without a qualifier is looked up in the globals first.
  A LIB helper that does `t = 0` or `q0 = ...` silently overwrites a top-level `t`/`q0` of the step
  script. Inside LIB functions, declare scratch variables `local t = 0` when the name could
  also be used at the top level.
- **Strings**: `str_cat(a, b)` takes exactly 2 operands (nest them for more). `textmsg(s1, s2)` takes
  at most 2. Both format floats with 6 decimals. Strings over 1023 characters throw. `textmsg` is
  our result channel (`psx_logs`), so keep it to a few lines per step, not one line per loop
  iteration.
- **Lists vs poses**: `[...]` is a list, `p[...]` is a pose. List arithmetic is element-wise
  (`[1,2,3] * v`, `0 - a`, `a + b`) [doc, untested on our controller; we write the components
  out]. `pose_add(p1, p2)` adds positions but *multiplies* rotations. Use
  `pose_trans(p, p[dx,dy,dz,0,0,0])` for an offset in the tool frame and `pose_add` for a
  base-frame offset. Rotations are rotation vectors (axis × angle), not Euler angles.
- **Every loop must use up robot time** [doc]: the controller runs at 500 Hz and a thread that
  computes without consuming time is stopped ("Lost communication with Controller" →
  protective stop). Each `while` iteration needs a `speedl(..., t)`, `sync()`, `sleep()` or a
  move. A polling loop that only reads `get_tcp_force()` needs `sync()`. Always pass `t` to
  `speedl` in a loop: without it, `speedl` returns as soon as the target speed is reached, so
  later iterations take no time.
- **Runtime exceptions abort the step mid-way and skip the closing `set_tcp`** [doc]. Watch
  for `get_inverse_kin` without a solution, `normalize([0,0,0])`, `pow` of a negative base with
  a non-integer exponent, `to_num` on a non-number, `append`/`extend` past a list's capacity,
  a list index or `get_actual_joint_positions_history(steps)` out of range, and a `str_cat`
  over 1023 characters. Guard these before the call.
- `halt` stops the program but skips the closing `set_tcp`. Prefer `return False` from the
  primitive so the closing lines still run.

## Force mode

Signature: `force_mode(task_frame, selection, wrench, type, limits)`.

- **The selection vector takes only 0 or 1** [seen: n = [0, 0.009, -1] from the 0.5° board tilt
  gave `invalid_argument_value`]. Pass exact axes and flatten tilted vectors.
- `task_frame` is relative to **base**. We use `p[0,0,0,0,0,0]` with `type = 2` (frame not
  transformed). With an identity frame, selection and wrench are in base axes.
- **`limits` has two meanings**: on compliant axes it is the max **speed**; on non-compliant
  axes it is the max **deviation** of the actual pose from the program-set one. Exceeding the
  deviation aborts the program (`force_mode_max_position_error`) [seen].
  - 0.01 on a "held" axis lets it drift up to 10 mm [seen: z moved 3.4 / 4.7 mm during the wiggle].
    For axes that must stay put, use ~0.001.
  - That abort is no force guard [seen 10-25 port 6: the held z target sank, ~70 N on the plug
    for seconds before the abort]. Check the force yourself in every loop iteration of a
    force-mode primitive (`|Fz| > ~15 N` or z drift > 1 mm → `end_force_mode()`, back off,
    `return False`).
- `wrench` values on non-compliant axes are ignored.
- **Entering**: no motion along the compliant axes and no hard deceleration just before. Add a
  `sleep(0.02)` or longer after a `stopl` and before `force_mode` [doc].
- **Damping and gain scaling are sticky** [doc]: `force_mode_set_damping` (default 0.005) and
  `force_mode_set_gain_scaling` (default 1, > 1 can go unstable) stay set until called again.
  The controller is shared, so set both at the top of a force-mode step if the behaviour
  matters.
- `end_force_mode()` before any `movel`/`movej`. It also ends when the program stops, but
  **never end a program while pressing** [seen: C153 protective stop at 40 N]. Back off to < 1 N
  first (`release`).
- **No x/y float with pins/plug in a hole** [seen 10-26: x/y-compliant seat with a stale F/T
  zero → `force_mode_singularity` FAULT, power off]. Straight position-controlled moves with a
  force cap (`seat_z`).

## Threads (for a force watchdog)

The port-6 overload needs a guard that also runs while the main thread is inside a `movel`
or a primitive without its own check. A thread can do that [doc, untested on our cell]:

```
global wd_trip = False
thread watchdog():
  while True:
    F = get_tcp_force()
    if norm(F[2]) > 15.0:
      wd_trip = True
    end
    sync()
  end
end
wd = run watchdog()
...  # primitives check wd_trip in their loops and back off
kill wd
```

- Threads take **no parameters**, and their return value is discarded. Exchange data through
  `global` variables only.
- The thread loop needs `sync()` (see the busy-loop rule above).
- Only the thread that started a thread can `join` it. `kill` also stops its child threads.
  Kill the watchdog before the closing lines.
- `enter_critical` / `exit_critical` protect shared variables, but `sleep`, `sync`, moves and
  socket reads inside the section let other threads run. Keep critical sections to plain
  assignments.
- Keep motion commands in the main thread. Let the watchdog set the flag, and let the main loop
  do `end_force_mode()`, `stopl` and the back-off. Use `protective_stop()` from the thread only
  when the main thread cannot react, such as in a long `movel` (the operator must then clear it).
- If you change the scoping or thread layout, lint and dry-run it once without contact first.

## F/T sensor and payload

- `get_tcp_force()` returns `p[Fx,Fy,Fz,Tx,Ty,Tz]`: forces **in base orientation**, measured at
  the **flange**, not the TCP. Dot it with base unit vectors directly. `force()` is its norm.
- **`set_target_payload(...)` also zeroes the F/T** [doc], and our preamble calls it
  (`urcore/urexec.py`, 1.5 kg). This explains why every program start re-zeroes the sensor
  [seen]: if a step starts while pressing, the press becomes the new zero. Start force steps at
  rest, out of contact.
- `zero_ftsensor()` subtracts the current reading. Call it at rest, then `sleep(0.3)`. Re-zero
  before every force step [seen: ~6 N offset after a pull, false contacts mid-air from drift].
  When zeroing while holding a seated part, the holding force is inside the zero.
- Use `set_target_payload(m, cog)` with the cog (the deprecated `set_payload` without a cog ties
  the CoG to the TCP). A wrong payload shows up as a few-N bias on compliant axes [seen: z rose
  while z was compliant at 0 N].

## TCP, poses, reachability

- **`set_tcp` persists after the program** [seen]. The bridge's `robot_get_pose`/IK and other
  people's programs then use our tip TCP. `URExec.run` appends `set_tcp(p[0,0,0,0,0,0])`, but a
  faulted step never gets there. After a fault, check the active TCP before trusting a pose.
- `get_actual_tool_flange_pose()` always returns the **flange** whatever TCP is active, and
  `get_tcp_offset()` returns the active TCP. Use them instead of guessing which TCP is set.
- `get_actual_tcp_pose()` = measured, `get_target_tcp_pose()` = commanded. Their difference is
  the force-mode deviation, the value that tells you a held axis is being pushed.
- **`get_inverse_kin` throws a runtime error when there is no solution** [doc]. Guard it with
  `get_inverse_kin_has_solution(pose, qnear)`. `is_within_safety_limits(pose, qnear)` also
  checks joint limits, safety planes and the tool orientation limit. Both use the active TCP
  unless you pass `tcp=`. Still reject |elbow| < 0.5 rad yourself [seen: straight elbow → C281
  FAULT].
- `movel` also accepts a joint list (FK is applied). `movel(..., t=...)` ignores `a` and `v`.
- On PolyScope X only motion version 2 exists (`motion_version_set` is not needed).

## Sockets (skill_b dispatcher)

- `socket_open(host, port, name)` gives up after 2 s and returns False. Check it and
  `textmsg` the failure.
- `socket_read_ascii_float(n, name, timeout)` expects `(a, b, c)` (parentheses, commas), at most
  30 values, and returns `[count, v1, ...]`. On a timeout it returns `[0, nan, ...]`, so always
  check `c[0]` before using the values. A timeout ≤ 0 blocks forever.
- `socket_read_line` is deprecated. `socket_read_string` caps a message at 1024 characters.
- `socket_send_line` strings are subject to the `str_cat` 1023-character limit, so send poses
  and forces in separate lines if they grow.

## Do not use on our shared cell

- `tool_wrench_limit_set`: persists until controller shutdown and slows every program.
- `popup(..., blocking=True)` / `request_boolean_from_primary_client`: blocks the program until
  someone presses the pendant, and the step's results never come back.
- `set_payload*` (deprecated), `teach_mode` (deprecated), secondary programs (`sec`: no motion,
  no sleep).
- Robotiq `rq_*` functions: they exist only in apps whose application has the Hand-E. Grip
  through the bridge smart skills instead (see `protoboard-wire`).

## Useful built-ins we under-use

- `tool_contact(direction, force_threshold)` returns the number of steps back to the contact
  start (0 = none). Combine it with `get_actual_joint_positions_history(steps)` to return to
  the exact first-contact pose.
- `protective_stop()` triggers a controlled protective stop from a watchdog. The operator
  must clear it, so use it only as a last resort.
- `position_deviation_warning(True, 0.5)` logs a warning before a deviation becomes a
  protective stop.
- `time()` gives monotonic controller time for real timeouts instead of summing step
  times. It returns a **struct**, not a number: `tt = time()` then `s = tt.sec + tt.nanosec / 1000000000`.
- `interpolate_pose(a, b, alpha)` for straight-line steps (e.g. the 0.1 mm relief ramp),
  `point_dist(a, b)` for the translation distance.
