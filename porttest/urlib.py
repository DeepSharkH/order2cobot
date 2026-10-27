"""URScript primitives of the switch / RJ45 task (creep, peck, search_port, engage*, seat_x,
seat_compliant*), appended to the shared primitives in urcore/urlib.py. LIB = both, which is
what porttest.urexec, skill_b's dispatcher and the simulator tests compile. Run
`python -m porttest.urscript_lint` after every change (and bump skill_b.VERSION)."""
from urcore.urlib import LIB as CORE_LIB

RJ45_LIB = '''
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

'''

LIB = CORE_LIB + RJ45_LIB
