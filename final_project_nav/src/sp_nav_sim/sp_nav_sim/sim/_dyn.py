from __future__ import annotations

from sp_nav_sim.sim._dyn_core import Plant as _Plant

class VelocityFilter:

    def __init__(self, dt: float):
        self._p = _Plant(float(dt))

    def reset(self) -> None:
        self._p.reset()

    def stopping_lag(self) -> float:
        return float(self._p.stopping_lag())

    def set_vel(self, vx: float, vy: float) -> None:
        self._p.set_vel(float(vx), float(vy))

    def step(
        self,
        vx_bl: float,
        vy_bl: float,
        yaw_bl: float,
        yaw_ch: float,
        wz_ch: float,
        v_max: float,
        a_max: float,
    ):
        return self._p.step(
            float(vx_bl), float(vy_bl),
            float(yaw_bl), float(yaw_ch), float(wz_ch),
            float(v_max), float(a_max),
        )
