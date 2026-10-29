"""
tests/fastsim.py - the full-system simulator (simulate.py) WITHOUT drawing camera pictures: the tracker's output is
replaced by the true positions plus noise, delay and dropouts. Everything else is the real code (pipeline, evasion,
planner, chase code, the real Micromelon Motors module on a simulated rover). About 20x faster, so it can be run many
times with different seeds and settings.
"""
import math
import os
import sys
import types

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import evade as ev
import sim_world as sw
import simulate as S
import tracker as tk


class _DummyRenderer:
    def __init__(self, *a, **k): pass


class _DummyTracker:
    def __init__(self, *a, **k): pass


class FastSim(S.Sim):
    def __init__(self, scenario, seed=0, cam_noise=0.008, yaw_noise_deg=2.0, dropout=0.0, cam_latency=None,
                 lidar_on=True, empirical=True, noise_scale=1.0, rho=0.85, **kw):
        self.cam_noise, self.yaw_noise, self.dropout = cam_noise, yaw_noise_deg, dropout
        self.empirical, self.noise_scale, self.rho = empirical, noise_scale, rho
        self._prev_err = {}
        _d = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "tracker_noise.npz"))
        self._pool = {"follower": _d["follower"], "opponent": _d["opponent"]}
        self.cam_latency = S.CAM_LATENCY if cam_latency is None else cam_latency
        self.lidar_on = lidar_on
        saved = (sw.Renderer, tk.Tracker)
        sw.Renderer, tk.Tracker = _DummyRenderer, _DummyTracker
        try:
            super().__init__(scenario, seed=seed, camera_drift=False, **kw)
        finally:
            sw.Renderer, tk.Tracker = saved
        self.nrng = np.random.default_rng(seed + 7)

    def _calibrate(self):                                   # only the lidar mask (own balloon) needs learning
        snap0 = self.world.snapshot_at(0.0)
        frames = [sw.lidar_frame(snap0, self.rng) for _ in range(40)]
        self.detector.learn_mask(frames)

    def _err(self, key, kind):
        """Tracker error for one object. 'empirical' replays errors measured from the full renderer (tracker_noise.npz),
        smoothed over time (AR(1)) because real errors are correlated from frame to frame."""
        n = self.nrng
        if not self.empirical:
            if kind == "follower":
                return np.array([n.normal(0, self.cam_noise), n.normal(0, self.cam_noise), n.normal(0, self.yaw_noise)])
            return np.array([n.normal(0, self.cam_noise), n.normal(0, self.cam_noise)])
        pool = self._pool[kind]
        draw = pool[n.integers(len(pool))] * self.noise_scale
        prev = self._prev_err.get(key)
        e = draw if prev is None else self.rho * prev + math.sqrt(1 - self.rho ** 2) * draw
        self._prev_err[key] = e
        return e

    def _state_from(self, sn):
        n = self.nrng
        st = {}
        x, y, yaw, _ = sn.agents["follower"]
        e = self._err("follower", "follower")
        st["follower"] = {"x": x + e[0], "y": y + e[1], "yaw": math.degrees(yaw) + e[2], "age": 0.0}
        for name, (ax, ay, ayaw, popped) in sn.agents.items():
            if name == "follower" or popped:
                continue
            if n.random() < self.dropout:
                continue
            bx, by = ax - sw.BALLOON_BACK * math.cos(ayaw), ay - sw.BALLOON_BACK * math.sin(ayaw)
            e = self._err(name, "opponent")
            st[name] = {"x": bx + e[0], "y": by + e[1], "age": 0.0}
        if n.random() < self.dropout:
            st.pop("follower")
        return st

    def tick(self):
        w = self.world
        w.step()
        t = w.t
        self.tick_n += 1
        if self.tick_n % 2 == 0:                                            # camera at 15 Hz
            sn = w.snapshot_at(t - self.cam_latency)
            self.last_state = self._state_from(sn)
            self.pipe.on_state(self.last_state, t)
            self.cam_log.append((self.last_state, sn.agents))
        if self.lidar_on:
            sn2 = w.snapshot_at(t - S.LIDAR_LATENCY)
            tap = self.motors.tap
            self.pipe.on_lidar(sw.lidar_frame(sn2, self.rng), sn2.t, tap.forward_mm_s, tap.rotating, t)
        self._log(t)
        if w.follower.popped:
            raise S.SimEnd("follower popped")
        if all(o.popped for o in w.opps):                             # the game is won: give the rover 3 s to notice and stop
            self.all_popped_t = getattr(self, "all_popped_t", t)
            if t - self.all_popped_t >= 3.0:
                raise S.SimEnd("all opponents popped")
        if self.scn.get("until") and self.scn["until"](self):
            raise S.SimEnd("goal reached")


def run(scenario, seed=0, duration=None, **kw):
    sim = FastSim(scenario, seed=seed, **kw)
    summary = sim.run(duration or scenario["duration"])
    return sim, summary
