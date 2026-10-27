"""URScript primitives that no live code calls any more (2026-10-27 split): the old grid /
profile / peck scans and the first insert versions, used only by the archived find_port*,
qa_run and touch scripts. They call tel, guarded_axial and peck (urcore.urlib / porttest.urlib)."""

LIB = '''
def insert(d, maxd, fmax, v):
  # push the plug home along d, then back off 0.5 mm so the program never ends while pressing
  # (ending at ~40 N into the port raised a protective stop, C153)
  hit = guarded_axial(d, maxd, fmax, v)
  p = get_actual_tcp_pose()
  movel(pose_add(p, p[-0.0005*d[0], -0.0005*d[1], -0.0005*d[2], 0, 0, 0]), a=0.1, v=0.002)
  textmsg("seated ", get_actual_tcp_pose())
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
'''
