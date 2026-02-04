# gym/f110_gym/envs/commonroad_rc_params.py
import numpy as np

def _safe_set(obj, name, value):
    if hasattr(obj, name):
        setattr(obj, name, value)

def _apply_tire_overrides(p, tire_dict):
    """
    Apply a dict of CommonRoad tire coefficients onto p.tire.
    Ignores keys that don't exist in the installed parameter class.
    """
    if not (hasattr(p, "tire") and p.tire is not None):
        return

    for k, v in tire_dict.items():
        _safe_set(p.tire, k, float(v))

def _default_rc_tire(mu=1.1):
    """
    Reasonable RC starting point for CommonRoad STD tire parameters.

    Philosophy:
    - keep mu explicit via p_dx1 / p_dy1
    - choose moderate shapes (C ~ 1.6..1.9)
    - choose stiffness via p_kx1, p_ky1 = B*C*mu (B absorbed into p_k* here)
    - keep offsets and camber terms ~0 unless you have data
    - use moderate combined-slip shaping so it doesn't kill longitudinal accel
    """
    mu = float(mu)

    # Pick C values (shape)
    Cx = 1.75
    Cy = 1.60

    # Choose "effective B" to set slope near zero: K/Fz = B*C*mu
    # Here we choose Bx~8.7, By~10.0 (RC tends to feel "snappy" at low slip)
    Bx = 8.7
    By = 10.0

    tire = {
        # --- longitudinal (pure slip)
        "p_cx1": Cx,
        "p_dx1": mu,
        "p_dx3": 0.0,
        "p_ex1": 0.20,
        "p_kx1": Bx * Cx * mu,
        "p_hx1": 0.0,
        "p_vx1": 0.0,

        # --- lateral (pure slip)
        "p_cy1": Cy,
        "p_dy1": mu,
        "p_dy3": 0.0,
        "p_ey1": 0.05,
        "p_ky1": By * Cy * mu,
        "p_hy1": 0.0,
        "p_hy3": 0.0,
        "p_vy1": 0.0,
        "p_vy3": 0.0,

        # --- combined slip (keep moderate)
        # Fx reduction with lateral slip
        "r_bx1": 8.0,
        "r_bx2": -8.0,
        "r_cx1": 1.20,
        "r_ex1": 0.50,
        "r_hx1": 0.0,

        # Fy reduction with longitudinal slip + small additional term
        "r_by1": 7.0,
        "r_by2": 8.0,
        "r_by3": 0.0,
        "r_cy1": 1.10,
        "r_ey1": -0.20,
        "r_hy1": 0.0,
        "r_vy1": 0.0,
        "r_vy3": 0.0,
        "r_vy4": 8.0,
        "r_vy5": 1.6,
        "r_vy6": -8.0,
    }
    return tire

def make_rc_commonroad_params(f110_params):
    """
    Builds a CommonRoad parameter object for STD, using:
      - vehicle scalars / constraints from f110_params
      - RC-tuned tire parameters (either user-provided or built-in defaults)

    Does NOT modify the installed library.
    """
    from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
    p = parameters_vehicle1()

    # --- vehicle scalars used by STD
    p.a   = float(f110_params["lf"])
    p.b   = float(f110_params["lr"])
    p.m   = float(f110_params["m"])
    p.I_z = float(f110_params["I"])

    _safe_set(p, "h_s", float(f110_params.get("h_cg", f110_params.get("h", getattr(p, "h_s", 0.02)))))
    _safe_set(p, "R_w", float(f110_params.get("wheel_radius", getattr(p, "R_w", 0.053))))
    _safe_set(p, "I_y_w", float(f110_params.get("I_y_w", 2.5e-4)))
    _safe_set(p, "T_sb", float(f110_params.get("T_sb", 0.5)))
    _safe_set(p, "T_se", float(f110_params.get("T_se", 0.5)))

    # --- constraints (if exposed by the CommonRoad parameter object)
    if hasattr(p, "steering") and p.steering is not None:
        _safe_set(p.steering, "min",   float(f110_params.get("s_min", -0.42)))
        _safe_set(p.steering, "max",   float(f110_params.get("s_max", +0.42)))
        _safe_set(p.steering, "v_min", float(f110_params.get("sv_min", -3.2)))
        _safe_set(p.steering, "v_max", float(f110_params.get("sv_max", +3.2)))

    if hasattr(p, "longitudinal") and p.longitudinal is not None:
        _safe_set(p.longitudinal, "v_min",    float(f110_params.get("v_min", -5.0)))
        _safe_set(p.longitudinal, "v_max",    float(f110_params.get("v_max", 20.0)))
        _safe_set(p.longitudinal, "v_switch", float(f110_params.get("v_switch", 2.0)))
        _safe_set(p.longitudinal, "a_max",    float(f110_params.get("a_max", 3.0)))
        _safe_set(p.longitudinal, "a_min",    float(f110_params.get("a_min", -3.0)))

    # --- tire: either user overrides or RC default
    mu = float(f110_params.get("mu", 1.1))

    # user can pass a full CommonRoad-tire dict as params["tire_rc"]
    tire_rc = f110_params.get("tire_rc", None)
    if tire_rc is None:
        tire_rc = _default_rc_tire(mu=mu)

    _apply_tire_overrides(p, tire_rc)
    return p
