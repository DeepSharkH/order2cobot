"""URScript primitives for any insertion task, prepended to every step (urcore.urexec).
Results are reported with textmsg(). Task-specific primitives live with their task and are
appended after this LIB (switch: porttest/urlib.py). Run `python -m urcore.urscript_lint`
after every change."""

LIB = '''
def tel(flim, phase):
  # telemetry for urcore.telemetry (RTDE): force limit in use and phase code
  write_output_float_register(24, flim)
  write_output_float_register(25, phase)
end

def fnorm(F):
  return sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2])
end

def guarded(d, maxd, fmax, v):
  tel(fmax, 9)
  # move along the unit vector d (base) at v m/s until |F| > fmax or maxd travelled
  zero_ftsensor()
  sleep(0.3)
  p0 = get_actual_tcp_pose()
  hit = False
  while True:
    speedl([d[0]*v, d[1]*v, d[2]*v, 0, 0, 0], 0.2, 0.008)
    F = get_tcp_force()
    if fnorm(F) > fmax:
      hit = True
      break
    end
    p = get_actual_tcp_pose()
    if point_dist(p, p0) > maxd:
      break
    end
  end
  stopl(1.0)
  textmsg("contact ", hit)
  textmsg("pose ", get_actual_tcp_pose())
  textmsg("force ", get_tcp_force())
  return hit
end

def guarded_axial(d, maxd, fmax, v):
  tel(fmax, 9)
  # like guarded(), but only the reaction against the motion counts (cable pull sideways is ignored)
  zero_ftsensor()
  sleep(0.5)
  p0 = get_actual_tcp_pose()
  hit = False
  while True:
    speedl([d[0]*v, d[1]*v, d[2]*v, 0, 0, 0], 0.2, 0.008)
    F = get_tcp_force()
    if -(F[0]*d[0] + F[1]*d[1] + F[2]*d[2]) > fmax:
      hit = True
      break
    end
    if point_dist(get_actual_tcp_pose(), p0) > maxd:
      break
    end
  end
  stopl(1.0)
  textmsg("contact ", hit)
  textmsg("pose ", get_actual_tcp_pose())
  textmsg("force ", get_tcp_force())
  return hit
end

def adv_along(n, ref):
  q = get_actual_tcp_pose()
  return q[0]*n[0] + q[1]*n[1] + q[2]*n[2] - ref
end

def along(q0, s):
  q = get_actual_tcp_pose()
  return (q[0]-q0[0])*s[0] + (q[1]-q0[1])*s[1] + (q[2]-q0[2])*s[2]
end

def seg(s, L, v, n, ref, adv):
  # slide along s for L (in-plane) unless the tip advances along n by > adv; time-capped
  q0 = get_actual_tcp_pose()
  t = 0
  while along(q0, s) < L and t < L / v * 3 + 1.0:
    speedl([s[0]*v, s[1]*v, s[2]*v, 0, 0, 0], 0.2, 0.008)
    t = t + 0.008
    if adv_along(n, ref) > adv:
      stopl(0.5)
      return True
    end
  end
  stopl(0.5)
  return False
end

def contact(n, fn):
  return contact_l(n, fn, [0.01, 0.005, 0.01, 0.1, 0.1, 0.1])
end

def contact_l(n, fn, lim):
  write_output_float_register(24, fn)   # force only: callers own the phase code
  # force mode pressing fn along n (tip must already be within ~1 mm of the face); waits
  # until the tip stops moving along n, returns the face coordinate along n.
  # lim: force_mode limits, base x y z order (compliant axis: max speed; others: max deviation)
  zero_ftsensor()
  sleep(0.3)
  force_mode(p[0, 0, 0, 0, 0, 0], [norm(n[0]), norm(n[1]), norm(n[2]), 0, 0, 0], [fn*n[0], fn*n[1], fn*n[2], 0, 0, 0], 2,
             lim)
  c0 = adv_along(n, 0)
  sleep(0.5)
  t = 0
  last = adv_along(n, 0)
  c = last
  while t < 6:
    sleep(0.2)
    t = t + 0.2
    c = adv_along(n, 0)
    if norm(c - last) < 0.00005:
      break
    end
    if c - c0 > 0.004:
      textmsg("no face")
      break
    end
    last = c
  end
  textmsg(str_cat("face ", c))
  return c
end

def found_face(n, c0, c):
  # contact() ran more than 4 mm without meeting the face
  return c - c0 <= 0.004
end

def back_off(n, d):
  tel(0, 7)
  end_force_mode()
  q = get_actual_tcp_pose()
  movel(pose_add(q, p[-d*n[0], -d*n[1], -d*n[2], 0, 0, 0]), a=0.1, v=0.005)
end

def spiral_search(n, fn, a, b, pitch, rmax, adv, v):
  return spiral_search_l(n, fn, a, b, pitch, rmax, adv, v, [0.01, 0.005, 0.01, 0.1, 0.1, 0.1])
end

def spiral_search_l(n, fn, a, b, pitch, rmax, adv, v, lim):
  tel(fn, 3)
  # contact, then a square spiral centre-out in a/b; stays in force mode on a hit
  c0 = adv_along(n, 0)
  ref = contact_l(n, fn, lim)
  if not found_face(n, c0, ref):
    back_off(n, 0.004)
    return False
  end
  return spiral_from(ref, n, a, b, pitch, rmax, adv, v)
end

def spiral_from(ref, n, a, b, pitch, rmax, adv, v):
  k = 1
  d = 0
  while k * pitch <= 2 * rmax:
    s = a
    if d % 4 == 1:
      s = b
    elif d % 4 == 2:
      s = [-a[0], -a[1], -a[2]]
    elif d % 4 == 3:
      s = [-b[0], -b[1], -b[2]]
    end
    if seg(s, k * pitch, v, n, ref, adv):
      textmsg("candidate ", get_actual_tcp_pose())
      return True
    end
    if d % 2 == 1:
      k = k + 1
    end
    d = d + 1
  end
  textmsg("not found")
  textmsg("pose ", get_actual_tcp_pose())
  back_off(n, 0.002)
  return False
end

def lateral(s, amp, v):
  return lateral_l(s, amp, v, 8.0)
end

def lateral_l(s, amp, v, fstop):
  # move amp along s (stop early above fstop), return the reaction force along s, come back
  q0 = get_actual_tcp_pose()
  t = 0
  while along(q0, s) < amp and t < amp / v * 3 + 0.5:
    speedl([s[0]*v, s[1]*v, s[2]*v, 0, 0, 0], 0.2, 0.008)
    t = t + 0.008
    F = get_tcp_force()
    if -(F[0]*s[0] + F[1]*s[1] + F[2]*s[2]) > fstop:
      break
    end
  end
  F = get_tcp_force()
  stopl(0.5)
  f = -(F[0]*s[0] + F[1]*s[1] + F[2]*s[2])
  movel(q0, a=0.1, v=0.003)
  return f
end

def wiggle_test(a, b, amp, fthr):
  return wiggle_test_l(a, b, amp, fthr, 8.0)
end

def wiggle_test_l(a, b, amp, fthr, fstop):
  tel(fthr, 5)
  # in force mode at a candidate: a wall on at least one side of a AND of b = inside the port.
  # Use amp >= 0.8 mm: the plug has ~0.3 mm play and sits off-centre (port 2, 10-25: at 0.3 mm
  # max 2.6 N, at 0.8 mm -a 5.05 N / +b 4.08 N with the opposite sides free)
  fa1 = lateral_l(a, amp, 0.002, fstop)
  fa2 = lateral_l([-a[0], -a[1], -a[2]], amp, 0.002, fstop)
  fb1 = lateral_l(b, amp, 0.002, fstop)
  fb2 = lateral_l([-b[0], -b[1], -b[2]], amp, 0.002, fstop)
  textmsg(str_cat("wiggle ", str_cat(str_cat(fa1, str_cat(" ", fa2)), str_cat(" ", str_cat(fb1, str_cat(" ", fb2))))))
  return (fa1 > fthr or fa2 > fthr) and (fb1 > fthr or fb2 > fthr)
end

def release(n, fmax, maxd):
  # leave force mode and back off along -n in 0.1 mm steps until the push along n is < fmax
  # (a fixed 0.5 mm back-off still left 12.3 N pressing on port 1); at most maxd
  tel(fmax, 7)
  end_force_mode()
  q0 = get_actual_tcp_pose()
  F = get_tcp_force()
  while -(F[0]*n[0] + F[1]*n[1] + F[2]*n[2]) > fmax and -along(q0, n) < maxd:
    q = get_actual_tcp_pose()
    movel(pose_add(q, p[-0.0001*n[0], -0.0001*n[1], -0.0001*n[2], 0, 0, 0]), a=0.1, v=0.002)
    sleep(0.1)
    F = get_tcp_force()
  end
  textmsg(str_cat(str_cat("released ", -along(q0, n)), str_cat(" Fn ", -(F[0]*n[0] + F[1]*n[1] + F[2]*n[2]))))
end

# --- pins into a board (protoboard-wire, breadboard). Poses are pins-TCP poses (set_tcp(tcp_pins)); n = -board normal. ---
def seat_z(depth, fmax, v):
  # protoboard pins: straight down along base -z only (x/y and rotation stay position-controlled,
  # no compliant axes: a x/y-compliant force mode drove the arm into a singularity on 10-26).
  # Stops after `depth` or when the push exceeds fmax, then backs off until the push is < 1 N.
  tel(fmax, 6)
  zero_ftsensor()
  sleep(0.3)
  q0 = get_actual_tcp_pose()
  s = [0, 0, -1]
  peak = 0
  t = 0
  f = 0
  while along(q0, s) < depth and t < depth / v * 3 + 1.0:
    speedl([0, 0, -v, 0, 0, 0], 0.2, 0.008)
    t = t + 0.008
    F = get_tcp_force()
    f = F[2]
    if f > peak:
      peak = f
    end
    if f > fmax:
      break
    end
  end
  stopl(0.5)
  d = along(q0, s)
  textmsg(str_cat(str_cat("seated_z ", d), str_cat(str_cat(" peak ", peak), str_cat(" last ", f))))
  textmsg("seatpose ", get_actual_tcp_pose())
  release(s, 1.0, 0.001)
  return d
end

def pull_out(n, fmax, maxd, v):
  # move along -n (out of the board) at v until maxd; stop if the pull exceeds fmax.
  # Reports the peak pull force (the pins' holding force) and the travel.
  tel(fmax, 8)
  zero_ftsensor()
  sleep(0.3)
  q0 = get_actual_tcp_pose()
  s = [-n[0], -n[1], -n[2]]
  peak = 0
  t = 0
  while along(q0, s) < maxd and t < maxd / v * 3 + 1.0:
    speedl([s[0]*v, s[1]*v, s[2]*v, 0, 0, 0], 0.2, 0.008)
    t = t + 0.008
    F = get_tcp_force()
    f = -(F[0]*s[0] + F[1]*s[1] + F[2]*s[2])
    if f > peak:
      peak = f
    end
    if f > fmax:
      stopl(0.5)
      textmsg(str_cat("pull limit ", f))
      textmsg("pose ", get_actual_tcp_pose())
      return False
    end
  end
  stopl(0.5)
  textmsg(str_cat(str_cat("pulled ", along(q0, s)), str_cat(" peak ", peak)))
  textmsg("pose ", get_actual_tcp_pose())
  return True
end
'''
