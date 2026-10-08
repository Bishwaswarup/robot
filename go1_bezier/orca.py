"""ORCA: Optimal Reciprocal Collision Avoidance (van den Berg et al., ISRR 2011).

A direct Python port of the 2D linear programs of the RVO2 library
(Agent::computeNewVelocity, linearProgram1/2/3). Static disc obstacles are handled
as non-moving agents that take no responsibility (their half-planes go first and are
treated as hard constraints in linearProgram3).

    v_new = orca_velocity(p, v, r, v_pref, v_max, neighbors, obstacles, tau, tau_obs, dt)
      neighbors: list of (p_j, v_j, r_j)    obstacles: list of (p_o, r_o)
"""
import numpy as np

EPS = 1e-5


def _det(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _norm(a):
    n = np.hypot(a[0], a[1])
    return a / n if n > 0 else a


def _half_plane(p, v, r, pj, vj, rj, tau, dt, responsibility):
    rel_pos = pj - p
    rel_vel = v - vj
    dist_sq = rel_pos @ rel_pos
    R = r + rj
    R_sq = R * R
    if dist_sq > R_sq:                                    # no collision yet
        inv_tau = 1.0 / tau
        w = rel_vel - inv_tau * rel_pos
        w_len_sq = w @ w
        dot1 = w @ rel_pos
        if dot1 < 0.0 and dot1 * dot1 > R_sq * w_len_sq:  # project on cut-off circle
            w_len = np.sqrt(w_len_sq)
            unit_w = w / w_len
            direction = np.array([unit_w[1], -unit_w[0]])
            u = (R * inv_tau - w_len) * unit_w
        else:                                             # project on legs
            leg = np.sqrt(dist_sq - R_sq)
            if _det(rel_pos, w) > 0.0:
                direction = np.array([rel_pos[0] * leg - rel_pos[1] * R,
                                      rel_pos[0] * R + rel_pos[1] * leg]) / dist_sq
            else:
                direction = -np.array([rel_pos[0] * leg + rel_pos[1] * R,
                                       -rel_pos[0] * R + rel_pos[1] * leg]) / dist_sq
            u = (rel_vel @ direction) * direction - rel_vel
    else:                                                 # already overlapping
        inv_dt = 1.0 / dt
        w = rel_vel - inv_dt * rel_pos
        w_len = np.hypot(w[0], w[1])
        unit_w = w / max(w_len, 1e-9)
        direction = np.array([unit_w[1], -unit_w[0]])
        u = (R * inv_dt - w_len) * unit_w
    return (v + responsibility * u, direction)


def _lp1(lines, k, radius, opt, dir_opt):
    pt, dr = lines[k]
    dot = pt @ dr
    disc = dot * dot + radius * radius - pt @ pt
    if disc < 0.0:
        return None
    s = np.sqrt(disc)
    t_left, t_right = -dot - s, -dot + s
    for i in range(k):
        denom = _det(dr, lines[i][1])
        numer = _det(lines[i][1], pt - lines[i][0])
        if abs(denom) <= EPS:
            if numer < 0.0:
                return None
            continue
        t = numer / denom
        if denom >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return None
    if dir_opt:
        return pt + (t_right if opt @ dr > 0.0 else t_left) * dr
    t = dr @ (opt - pt)
    return pt + (t_left if t < t_left else (t_right if t > t_right else t)) * dr


def _lp2(lines, radius, opt, dir_opt):
    if dir_opt:
        result = opt * radius
    elif opt @ opt > radius * radius:
        result = _norm(opt) * radius
    else:
        result = opt.copy()
    for i in range(len(lines)):
        if _det(lines[i][1], lines[i][0] - result) > 0.0:
            r = _lp1(lines, i, radius, opt, dir_opt)
            if r is None:
                return i, result
            result = r
    return len(lines), result


def _lp3(lines, n_obst, begin, radius, result):
    distance = 0.0
    for i in range(begin, len(lines)):
        if _det(lines[i][1], lines[i][0] - result) > distance:
            proj = list(lines[:n_obst])
            for j in range(n_obst, i):
                det = _det(lines[i][1], lines[j][1])
                if abs(det) <= EPS:
                    if lines[i][1] @ lines[j][1] > 0.0:
                        continue
                    point = 0.5 * (lines[i][0] + lines[j][0])
                else:
                    point = lines[i][0] + (_det(lines[j][1], lines[i][0] - lines[j][0]) / det) * lines[i][1]
                proj.append((point, _norm(lines[j][1] - lines[i][1])))
            n, r = _lp2(proj, radius, np.array([-lines[i][1][1], lines[i][1][0]]), True)
            if n >= len(proj):
                result = r
            distance = _det(lines[i][1], lines[i][0] - result)
    return result


def orca_velocity(p, v, r, v_pref, v_max, neighbors=(), obstacles=(), tau=2.0, tau_obs=1.0, dt=0.1):
    p, v, v_pref = (np.asarray(x, float) for x in (p, v, v_pref))
    lines = [_half_plane(p, v, r, np.asarray(po, float), np.zeros(2), ro, tau_obs, dt, 1.0)
             for po, ro in obstacles]
    n_obst = len(lines)
    lines += [_half_plane(p, v, r, np.asarray(pj, float), np.asarray(vj, float), rj, tau, dt, 0.5)
              for pj, vj, rj in neighbors]
    fail, result = _lp2(lines, v_max, v_pref, False)
    if fail < len(lines):
        result = _lp3(lines, n_obst, fail, v_max, result)
    return result
