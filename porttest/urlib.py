"""URScript helpers prepended to every step. Results are reported with textmsg()."""

LIB = '''
def tel(flim, phase):
  # telemetry for porttest.telemetry (RTDE): force limit in use and phase code
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

def insert(d, maxd, fmax, v):
  # push the plug home along d, then back off 0.5 mm so the program never ends while pressing
  # (ending at ~40 N into the port raised a protective stop, C153)
  hit = guarded_axial(d, maxd, fmax, v)
  p = get_actual_tcp_pose()
  movel(pose_add(p, p[-0.0005*d[0], -0.0005*d[1], -0.0005*d[2], 0, 0, 0]), a=0.1, v=0.002)
  textmsg("seated ", get_actual_tcp_pose())
  return hit
end

def creep(d, maxd, step, fmax, v):
  # advance along d in short pecks, re-zeroing before each, so the cable's spring pull never
  # builds up to the threshold. Stops at the first peck that ends short (contact).
  p0 = get_actual_tcp_pose()
  hit = False
  while point_dist(get_actual_tcp_pose(), p0) < maxd:
    t = peck(d, step, fmax, v)
    if t < step - 0.0003:
      hit = True
      break
    end
  end
  textmsg("creep contact ", hit)
  textmsg("pose ", get_actual_tcp_pose())
  return hit
end

def insert_fm(fy, depth, tmax):
  tel(fy, 6)
  # seat the plug with the controller's force mode: constant push fy [N] along base +y,
  # x/z held (max 1 mm deviation), speed capped at 10 mm/s. A speed-driven push into the
  # contacts lagged behind its path and raised C153 protective stops.
  sleep(0.2)
  zero_ftsensor()
  sleep(0.3)
  p0 = get_actual_tcp_pose()
  force_mode(p[0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0.0, fy, 0.0, 0.0, 0.0, 0.0], 2,
             [0.001, 0.01, 0.001, 0.05, 0.05, 0.05])
  t = 0
  while t < tmax:
    if get_actual_tcp_pose()[1] - p0[1] > depth:
      break
    end
    sync()
    t = t + get_steptime()
  end
  end_force_mode()
  stopl(0.5)
  q = get_actual_tcp_pose()
  textmsg(str_cat(str_cat("seated dy ", q[1] - p0[1]), str_cat(" t ", t)))
  return q[1] - p0[1]
end

def peck(d, maxd, fmax, v):
  tel(fmax, 9)
  # guarded move; the F/T sensor is re-zeroed at rest first (its reading drifts by several N
  # when the arm moves between pecks). Returns the distance travelled (maxd = no contact).
  sleep(0.2)
  zero_ftsensor()
  sleep(0.2)
  p0 = get_actual_tcp_pose()
  while True:
    speedl([d[0]*v, d[1]*v, d[2]*v, 0, 0, 0], 0.3, 0.008)
    F = get_tcp_force()
    if -(F[0]*d[0] + F[1]*d[1] + F[2]*d[2]) > fmax:   # reaction against the motion only
      break
    end
    if point_dist(get_actual_tcp_pose(), p0) > maxd:
      break
    end
  end
  stopl(1.0)
  return point_dist(get_actual_tcp_pose(), p0)
end

def grid_search(c, d, n, nx, sx, z0, nz, sz, back, depth, fmax):
  # c: tip pose on the face at the port estimate; d: unit face direction, n: unit face
  # normal into the switch (base). Columns centre-out along d, rows bottom-up in z.
  zero_ftsensor()
  sleep(0.3)
  i = 0
  while i < nx:
    k = floor((i + 1) / 2)
    ox = k * sx
    if i % 2 == 1:
      ox = -ox
    end
    j = 0
    while j < nz:
      oz = z0 + j * sz
      movel(pose_add(c, p[ox*d[0] - back*n[0], ox*d[1] - back*n[1], oz, 0, 0, 0]), a=0.2, v=0.01)
      t = peck(n, back + depth, fmax, 0.006)
      if t > back + 0.003:
        textmsg(str_cat(str_cat("found ", ox), str_cat(" ", oz)))
        textmsg("pose ", get_actual_tcp_pose())
        return True
      end
      j = j + 1
    end
    textmsg(str_cat("col done ", ox))
    i = i + 1
  end
  movel(pose_add(c, p[-0.01*n[0], -0.01*n[1], 0, 0, 0, 0]), a=0.2, v=0.01)
  textmsg("not found")
  return False
end

def profile(c, step, n, back, depth, fmax):
  # pecks along +y from fixed absolute starts c + i*step - back*y (no creeping forward);
  # reports the travel of each peck: < back + ~0.5 mm = face, ~ back + depth = free
  zero_ftsensor()
  sleep(0.3)
  i = 0
  while i < n:
    s = pose_add(c, p[i*step[0], i*step[1] - back, i*step[2], 0, 0, 0])
    movel(s, a=0.2, v=0.01)
    t = peck([0, 1, 0], back + depth, fmax, 0.006)
    F = get_tcp_force()
    textmsg(str_cat(str_cat("pf ", i), str_cat(str_cat(" z ", s[2]), str_cat(str_cat(" t ", t), str_cat(" Fy ", F[1])))))
    movel(s, a=0.2, v=0.01)
    i = i + 1
  end
end

def peck_scan(step, n, back, depth, fmax, nfree):
  # from a pose touching the face (+y): back off, shift to start + i*step (absolute, so
  # x/z do not drift), push +y again. Stops after `nfree` consecutive pecks that go
  # deeper than back + 2 mm.
  zero_ftsensor()
  sleep(0.3)
  s0 = get_actual_tcp_pose()
  free = 0
  i = 1
  while i <= n:
    p = get_actual_tcp_pose()
    movel(p[s0[0] + i*step[0], p[1] - back, s0[2] + i*step[2], s0[3], s0[4], s0[5]], a=0.2, v=0.01)
    t = peck([0, 1, 0], back + depth, fmax, 0.006)
    q = get_actual_tcp_pose()
    textmsg(str_cat(str_cat(str_cat("pk ", i), str_cat(" ", q[0])), str_cat(str_cat(" ", q[1]), str_cat(" ", q[2]))))
    if t > back + 0.002:
      free = free + 1
      if free >= nfree:
        break
      end
    else:
      free = 0
    end
    i = i + 1
  end
end
# --- slide-in-contact insertion (rj45-insert skill). Poses are plug-tip poses (set_tcp). ---
# n: unit face normal into the switch; a, b: in-plane unit vectors (base).

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

def slide_until_advance(n, fn, s, maxd, adv, v):
  tel(fn, 9)
  c0 = adv_along(n, 0)
  ref = contact(n, fn)
  if not found_face(n, c0, ref):
    back_off(n, 0.004)
    return False
  end
  hit = seg(s, maxd, v, n, ref, adv)
  textmsg(str_cat("advance ", hit))
  textmsg("pose ", get_actual_tcp_pose())
  back_off(n, 0.002)
  return hit
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

def search_port(n, fn, face, ain, a, b, pitch, rmax, adv, v):
  tel(fn, 3)
  # like spiral_search, but with the touched face coordinate `face` along n: if the first
  # contact already sits > ain past it, the tip entered the opening at the start point
  # (port 1, 10-25: stalled 1.1-1.6 mm in at 4 N) -> candidate, still in force mode
  ref = contact(n, fn)
  if ref - face > ain:
    textmsg(str_cat("entered at start ", ref - face))
    textmsg("candidate ", get_actual_tcp_pose())
    return True
  end
  if ref - face < -0.002:
    textmsg("face not where expected")
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

def seat_compliant(n, fy, depth, tmax):
  return seat_compliant_l(n, fy, depth, tmax, [0.003, 0.004, 0.003, 0.1, 0.1, 0.1], 3.0)
end

def seat_compliant_l(n, fy, depth, tmax, lim, frel):
  tel(fy, 6)
  # the in-plane axes float at zero force (self-align), push fy along n; then release until the
  # push is < frel. lim: force_mode speed limits, base x y z (the push axis: the seat speed)
  end_force_mode()
  q0 = get_actual_tcp_pose()
  r0 = q0[0]*n[0] + q0[1]*n[1] + q0[2]*n[2]
  force_mode(p[0, 0, 0, 0, 0, 0], [1, 1, 1, 0, 0, 0], [fy*n[0], fy*n[1], fy*n[2], 0, 0, 0], 2,
             lim)
  t = 0
  while t < tmax:
    if adv_along(n, r0) > depth:
      break
    end
    sync()
    t = t + get_steptime()
  end
  dy = adv_along(n, r0)
  textmsg("seatpose ", get_actual_tcp_pose())
  release(n, frel, 0.001)
  textmsg(str_cat("seated ", str_cat(dy, str_cat(" t ", t))))
  return dy
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

def engage(n, fe, face, depth, tmax):
  return engage_l(n, fe, face, depth, tmax, [0.003, 0.004, 0.003, 0.1, 0.1, 0.1], 4.0, [0.01, 0.005, 0.01, 0.1, 0.1, 0.1])
end

def engage_l(n, fe, face, depth, tmax, lim, fh, lim_hold):
  # from a candidate: push fe along n with the in-plane axes compliant (self-centring in the
  # mouth) until the tip is `depth` past the measured face or tmax; 4 N stalls at the mouth
  # (port 1: 1.1 mm in). Ends in force mode pushing fh along n only (in-plane stiff again, held
  # within lim_hold, for wiggle_test). lim / lim_hold: force_mode limits, base x y z
  tel(fe, 4)
  end_force_mode()
  force_mode(p[0, 0, 0, 0, 0, 0], [1, 1, 1, 0, 0, 0], [fe*n[0], fe*n[1], fe*n[2], 0, 0, 0], 2,
             lim)
  t = 0
  while t < tmax and adv_along(n, face) < depth:
    sync()
    t = t + get_steptime()
  end
  d = adv_along(n, face)
  end_force_mode()
  force_mode(p[0, 0, 0, 0, 0, 0], [norm(n[0]), norm(n[1]), norm(n[2]), 0, 0, 0], [fh*n[0], fh*n[1], fh*n[2], 0, 0, 0], 2,
             lim_hold)
  sleep(0.3)
  textmsg(str_cat(str_cat("engaged ", d), str_cat(" t ", t)))
  textmsg("engpose ", get_actual_tcp_pose())
  return d
end

def seat_x(n, fy, depth, tmax):
  # like seat_compliant, but z stays stiff (max 2 mm deviation): with z free the force loop's
  # ~2 N vertical bias lifted the taped switch 7 mm during the port-4 seat. x stays compliant.
  tel(fy, 6)
  end_force_mode()
  q0 = get_actual_tcp_pose()
  r0 = q0[0]*n[0] + q0[1]*n[1] + q0[2]*n[2]
  force_mode(p[0, 0, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0], [fy*n[0], fy*n[1], fy*n[2], 0, 0, 0], 2,
             [0.003, 0.004, 0.002, 0.1, 0.1, 0.1])
  t = 0
  while t < tmax:
    if adv_along(n, r0) > depth:
      break
    end
    sync()
    t = t + get_steptime()
  end
  dy = adv_along(n, r0)
  textmsg("seatpose ", get_actual_tcp_pose())
  release(n, 3.0, 0.001)
  textmsg(str_cat("seated ", str_cat(dy, str_cat(" t ", t))))
  return dy
end

def engage_x(n, fe, face, depth, tmax):
  # engage with z stiff (2 mm deviation), x compliant: the x/z-compliant engage let z climb
  # 3.9 mm and jam at 3.97 mm (port 2, 21:10). Ends in the 4 N n-only force mode like engage
  tel(fe, 4)
  end_force_mode()
  force_mode(p[0, 0, 0, 0, 0, 0], [1, 1, 0, 0, 0, 0], [fe*n[0], fe*n[1], fe*n[2], 0, 0, 0], 2,
             [0.003, 0.004, 0.002, 0.1, 0.1, 0.1])
  t = 0
  while t < tmax and adv_along(n, face) < depth:
    sync()
    t = t + get_steptime()
  end
  d = adv_along(n, face)
  end_force_mode()
  force_mode(p[0, 0, 0, 0, 0, 0], [norm(n[0]), norm(n[1]), norm(n[2]), 0, 0, 0], [4.0*n[0], 4.0*n[1], 4.0*n[2], 0, 0, 0], 2,
             [0.01, 0.005, 0.01, 0.1, 0.1, 0.1])
  sleep(0.3)
  textmsg(str_cat(str_cat("engaged ", d), str_cat(" t ", t)))
  textmsg("engpose ", get_actual_tcp_pose())
  return d
end

# --- protoboard-wire skill. Poses are pins-TCP poses (set_tcp(tcp_pins)); n = -board normal. ---

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
