"""
bluerov2heavy_dob_model.py
==========================
AcadosModel for the BlueROV2 Heavy — DOBMPC variant.

This is the model used by generate_c_code_dob.py to generate the
ACADOS C solver (bluerov2heavy_acados_*).

PARAMETERS (6):  sym_p = [Xw, Yw, Zw, Kw, Mw, Nw]
  Disturbances estimated by the EAOB (EKF) are injected here at runtime
  via bluerov2heavy_acados_update_params().  This is exactly the same
  pattern as the standard bluerov2.py but expanded to 6-DOF heavy.

CONTROLS (8):    sym_u = [u1..u8]  — direct thruster forces [N]
  No inverse allocation step; K (6×8) maps them straight to body forces.

PHYSICS:         Ng (2024) heavy-config parameters.
  Piecewise heave NOT used here (CasADi if_else is not supported as a
  runtime ACADOS parameter; use the nominal dive value for the solver
  and let the disturbance observer handle the residual).
"""

from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, fabs
import numpy as np

def export_bluerov2heavy_dob_model() -> AcadosModel:

    model_name = 'bluerov2heavy'

    # ── States (12) ────────────────────────────────────────────────────────────
    x     = SX.sym('x')
    y     = SX.sym('y')
    z     = SX.sym('z')
    phi   = SX.sym('phi')
    theta = SX.sym('theta')
    psi   = SX.sym('psi')
    u     = SX.sym('u')
    v     = SX.sym('v')
    w     = SX.sym('w')
    p     = SX.sym('p')
    q     = SX.sym('q')
    r     = SX.sym('r')
    sym_x = vertcat(x, y, z, phi, theta, psi, u, v, w, p, q, r)

    # ── Controls (8 — direct thruster forces [N]) ─────────────────────────────
    u1 = SX.sym('u1'); u2 = SX.sym('u2')
    u3 = SX.sym('u3'); u4 = SX.sym('u4')
    u5 = SX.sym('u5'); u6 = SX.sym('u6')
    u7 = SX.sym('u7'); u8 = SX.sym('u8')
    sym_u = vertcat(u1, u2, u3, u4, u5, u6, u7, u8)

    # ── Runtime parameters (6 — disturbances, updated by EAOB each step) ──────
    # Index mapping must match BLUEROV2HEAVY_NP = 6 in the header,
    # and acados_param[i][0..5] in bluerov2heavy_dob.cpp solve().
    disturbance_x     = SX.sym('disturbance_x')      # [0] Xw  [N]
    disturbance_y     = SX.sym('disturbance_y')      # [1] Yw  [N]
    disturbance_z     = SX.sym('disturbance_z')      # [2] Zw  [N]
    disturbance_phi   = SX.sym('disturbance_phi')    # [3] Kw  [Nm]
    disturbance_theta = SX.sym('disturbance_theta')  # [4] Mw  [Nm]
    disturbance_psi   = SX.sym('disturbance_psi')    # [5] Nw  [Nm]
    sym_p = vertcat(disturbance_x, disturbance_y, disturbance_z,
                    disturbance_phi, disturbance_theta, disturbance_psi)

    # ── State derivatives (for implicit residual f_impl) ──────────────────────
    x_dot     = SX.sym('x_dot');   y_dot     = SX.sym('y_dot')
    z_dot     = SX.sym('z_dot');   phi_dot   = SX.sym('phi_dot')
    theta_dot = SX.sym('theta_dot'); psi_dot = SX.sym('psi_dot')
    u_dot     = SX.sym('u_dot');   v_dot     = SX.sym('v_dot')
    w_dot     = SX.sym('w_dot');   p_dot     = SX.sym('p_dot')
    q_dot     = SX.sym('q_dot');   r_dot     = SX.sym('r_dot')
    sym_xdot  = vertcat(x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot,
                        u_dot, v_dot, w_dot, p_dot, q_dot, r_dot)

    # =========================================================================
    # PHYSICAL PARAMETERS  (Ng 2024 / Blue Robotics heavy spec)
    # =========================================================================
    m        = 11.5      # kg
    Ix       = 0.8571    # kg·m²
    Iy       = 1.0
    Iz       = 1.0
    ZG       = 0.02      # m  (CG below CB along body-z)
    g        = 9.81      # m/s²
    buoyancy = 6.675     # N  (net upward, ENU)

    # Added mass (Ng 2024 Table 4.1) — nominal dive value for heave
    X_udot = 10.77;  Y_vdot = 24.86;  Z_wdot = 34.60
    K_pdot = 0.103;  M_qdot = 0.120;  N_rdot = 0.120

    # Effective inertia
    M_x   = m + X_udot   # 22.27
    M_y   = m + Y_vdot   # 36.36
    M_z   = m + Z_wdot   # 46.10
    M_phi = Ix + K_pdot  # 0.9601
    M_th  = Iy + M_qdot  # 1.120
    M_psi = Iz + N_rdot  # 1.120

    # Linear damping (Ng 2024) — K_p, M_q, N_r experimentally zero
    X_u = 38.95;  Y_v = 60.11;  Z_w = 49.50
    K_p = 0.0;    M_q = 0.0;    N_r = 0.0

    # Quadratic damping (Ng 2024)
    X_uu = 31.01;  Y_vv = 36.95;  Z_ww = 113.86
    K_pp = 2.08;   M_qq = 2.42;   N_rr = 2.42

    # =========================================================================
    # THRUSTER ALLOCATION  K (6×8)
    # Rows: [X, Y, Z, K(roll), M(pitch), N(yaw)]
    # Cols: thrusters 0-7
    # =========================================================================
    K = np.array([
        [ 0.707,  0.707, -0.707, -0.707,  0.000,  0.000,  0.000,  0.000],
        [ 0.707, -0.707,  0.707, -0.707,  0.000,  0.000,  0.000,  0.000],
        [ 0.000,  0.000,  0.000,  0.000,  1.000,  1.000,  1.000,  1.000],
        [ 0.067, -0.067,  0.067, -0.067, -0.218,  0.218, -0.218,  0.218],
        [-0.067, -0.067,  0.067,  0.067, -0.120, -0.120,  0.120,  0.120],
        [ 0.1888,-0.1888,-0.1888, 0.1888,  0.000,  0.000,  0.000,  0.000],
    ])

    t  = [u1, u2, u3, u4, u5, u6, u7, u8]
    Kt = [sum(K[i, j] * t[j] for j in range(8)) for i in range(6)]

    # =========================================================================
    # EQUATIONS OF MOTION
    # Disturbance terms injected directly from runtime parameters sym_p.
    # =========================================================================

    # Translational
    du = (Kt[0] + m*r*v - m*q*w - buoyancy*sin(theta)
          - X_u*u - X_uu*u*fabs(u) + disturbance_x) / M_x

    dv = (Kt[1] - m*r*u + m*p*w + buoyancy*cos(theta)*sin(phi)
          - Y_v*v - Y_vv*v*fabs(v) + disturbance_y) / M_y

    dw = (Kt[2] + m*q*u - m*p*v + buoyancy*cos(theta)*cos(phi)
          - Z_w*w - Z_ww*w*fabs(w) + disturbance_z) / M_z

    # Rotational
    dp = (Kt[3] + (Iy-Iz)*q*r - m*ZG*g*cos(theta)*sin(phi)
          - K_p*p - K_pp*p*fabs(p) + disturbance_phi) / M_phi

    dq = (Kt[4] + (Iz-Ix)*p*r - m*ZG*g*sin(theta)
          - M_q*q - M_qq*q*fabs(q) + disturbance_theta) / M_th

    dr = (Kt[5] - (Iy-Ix)*p*q
          - N_r*r - N_rr*r*fabs(r) + disturbance_psi) / M_psi

    # Kinematics
    dx = ( cos(psi)*cos(theta))*u \
       + (-sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi))*v \
       + ( sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta))*w

    dy = ( sin(psi)*cos(theta))*u \
       + ( cos(psi)*cos(phi) + sin(phi)*sin(theta)*sin(psi))*v \
       + (-cos(psi)*sin(phi) + sin(theta)*sin(psi)*cos(phi))*w

    dz = (-sin(theta))*u \
       + ( cos(theta)*sin(phi))*v \
       + ( cos(theta)*cos(phi))*w

    dphi   = p + (sin(phi)*sin(theta)/cos(theta))*q + (cos(phi)*sin(theta)/cos(theta))*r
    dtheta =       cos(phi)*q - sin(phi)*r
    dpsi   = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r

    f_expl = vertcat(dx, dy, dz, dphi, dtheta, dpsi, du, dv, dw, dp, dq, dr)
    f_impl = sym_xdot - f_expl

    cost_y_expr = vertcat(sym_x, sym_u)

    model = AcadosModel()
    model.f_impl_expr   = f_impl
    model.f_expl_expr   = f_expl
    model.x             = sym_x
    model.xdot          = sym_xdot
    model.u             = sym_u
    model.p             = sym_p
    model.cost_y_expr   = cost_y_expr
    model.cost_y_expr_e = sym_x
    model.name          = model_name

    return model