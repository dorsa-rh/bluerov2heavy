"""
bluerov2heavy.py  —  AcadosModel for the BlueROV2 Heavy (8-thruster configuration)

BASELINE (all flags False) reproduces the repository model exactly.
Each flag is independently toggleable so you can isolate the effect of each change.

FLAGS
-----
STEP_1_NG2024_DAMPING    — Corrected linear damping from Ng (2024) water-tank experiments.
                           Main effect: sets K_p = M_q = N_r = 0 (rotational linear drag
                           is experimentally zero), adjusts X_u/Y_v/Z_w to measured values.
                           Safe to enable; purely a parameter correction.

STEP_2_QUADRATIC_DAMPING — Adds D|v|v drag terms for all 6 DOF (Ng 2024, Table 4.1).
                           Main effect: prevents MPC from underestimating drag at speed,
                           which caused the baseline to command insufficient thrust when
                           tracking fast trajectories. Smooth function; no solver risk.

STEP_3_PIECEWISE_HEAVE   — Asymmetric added-mass and drag for dive (w<0) vs rise (w>0),
                           reflecting the physical top/bottom asymmetry of the vehicle
                           (Ng 2024, Section 2.2.3, Figure 2.3).
                           CAVEAT: the CasADi if_else creates a non-smooth switch at w=0.
                           SQP_RTI handles this in practice, but watch solver status near
                           depth reversals. Disable first if you see acados status != 0.

COORDINATE CONVENTION
---------------------
Inertial frame : ENU  (East-North-Up)
Body frame     : FPU  (Forward-Port-Up)
Positive heave w : upward (ENU z-body direction).
  w < 0  →  vehicle diving  (moving downward)
  w > 0  →  vehicle rising  (moving upward)
"""

from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, fabs, if_else
import numpy as np

# ======================================================================
# TOGGLE FLAGS  —  set all False for the exact repository baseline model
# ======================================================================
STEP_1_NG2024_DAMPING    = False   # corrected linear damping (Ng 2024)
STEP_2_QUADRATIC_DAMPING = False   # add quadratic drag D|v|v  (Ng 2024)
STEP_3_PIECEWISE_HEAVE   = False   # asymmetric dive / rise heave (Ng 2024)


def export_bluerov2heavy_model() -> AcadosModel:

    model_name = 'bluerov2heavy'

    # ------------------------------------------------------------------
    # States  x ∈ R^12
    # ------------------------------------------------------------------
    x     = SX.sym('x')        # world position x   [m]
    y     = SX.sym('y')        # world position y   [m]
    z     = SX.sym('z')        # world position z   [m]
    phi   = SX.sym('phi')      # roll               [rad]
    theta = SX.sym('theta')    # pitch              [rad]
    psi   = SX.sym('psi')      # yaw                [rad]
    u     = SX.sym('u')        # surge velocity     [m/s]
    v     = SX.sym('v')        # sway velocity      [m/s]
    w     = SX.sym('w')        # heave velocity     [m/s]
    p     = SX.sym('p')        # roll rate          [rad/s]
    q     = SX.sym('q')        # pitch rate         [rad/s]
    r     = SX.sym('r')        # yaw rate           [rad/s]
    sym_x = vertcat(x, y, z, phi, theta, psi, u, v, w, p, q, r)

    # ------------------------------------------------------------------
    # Controls  u_ctrl ∈ R^8  (thruster forces [N])
    # ------------------------------------------------------------------
    u1 = SX.sym('u1'); u2 = SX.sym('u2')
    u3 = SX.sym('u3'); u4 = SX.sym('u4')
    u5 = SX.sym('u5'); u6 = SX.sym('u6')
    u7 = SX.sym('u7'); u8 = SX.sym('u8')
    sym_u = vertcat(u1, u2, u3, u4, u5, u6, u7, u8)

    # ------------------------------------------------------------------
    # State derivatives (needed for implicit DAE residual f_impl)
    # ------------------------------------------------------------------
    x_dot     = SX.sym('x_dot')
    y_dot     = SX.sym('y_dot')
    z_dot     = SX.sym('z_dot')
    phi_dot   = SX.sym('phi_dot')
    theta_dot = SX.sym('theta_dot')
    psi_dot   = SX.sym('psi_dot')
    u_dot     = SX.sym('u_dot')
    v_dot     = SX.sym('v_dot')
    w_dot     = SX.sym('w_dot')
    p_dot     = SX.sym('p_dot')
    q_dot     = SX.sym('q_dot')
    r_dot     = SX.sym('r_dot')
    sym_xdot  = vertcat(x_dot, y_dot, z_dot,
                        phi_dot, theta_dot, psi_dot,
                        u_dot, v_dot, w_dot,
                        p_dot, q_dot, r_dot)

    # ==================================================================
    # PHYSICAL PARAMETERS
    # Rigid-body values: Ng (2024) Table 4.1 / Blue Robotics spec sheet
    # ==================================================================
    m        = 11.5     # vehicle mass                      [kg]
    Ix       = 0.8571   # roll inertia                      [kg·m²]
    Iy       = 1.0      # pitch inertia                     [kg·m²]
    Iz       = 1.0      # yaw inertia                       [kg·m²]
    ZG       = 0.02     # CB above CG along body z-axis     [m]
    g        = 9.81     # gravitational acceleration         [m/s²]
    buoyancy = 6.675    # net buoyancy force (ENU, upward+)  [N]

    # ------------------------------------------------------------------
    # Added mass  (Ng 2024 Table 4.1 — same in baseline and improved)
    # ------------------------------------------------------------------
    X_udot = 10.77   # surge    [kg]
    Y_vdot = 24.86   # sway     [kg]
    K_pdot = 0.103   # roll     [kg·m²/rad]
    M_qdot = 0.120   # pitch    [kg·m²/rad]
    N_rdot = 0.120   # yaw      [kg·m²/rad]

    # Heave added mass — piecewise (Step 3) or single baseline value
    if STEP_3_PIECEWISE_HEAVE:
        # Ng (2024) Table 4.1:  dive (w<0 in ENU) = 34.60,  rise (w>0) = 22.45
        Z_wdot = if_else(w < 0, 34.60, 22.45)
    else:
        Z_wdot = 34.60      # baseline: dive value (Ng 2024 / repository)

    # ------------------------------------------------------------------
    # Linear damping
    # ------------------------------------------------------------------
    if STEP_1_NG2024_DAMPING:
        # Ng (2024) Table 4.1 — experimentally determined
        # Rotational linear damping is zero (least-squares confirmed Nr=0;
        # pitch and roll scaled proportionally give the same conclusion).
        X_u = 38.95
        Y_v = 60.11
        K_p = 0.0
        M_q = 0.0
        N_r = 0.0
        if STEP_3_PIECEWISE_HEAVE:
            Z_w = if_else(w < 0, 49.50, 50.62)
        else:
            Z_w = 49.50     # dive value; rise (50.62) differs by <3%
    else:
        # Repository baseline values
        X_u = 38.0
        Y_v = 60.0
        Z_w = 50.0
        K_p = 0.07
        M_q = 0.07
        N_r = 0.07

    # ------------------------------------------------------------------
    # Quadratic damping — zero in baseline, Ng (2024) if Step 2 enabled
    # The fabs() function is differentiable everywhere except at 0, where
    # the subgradient is 0; SQP_RTI handles this without issue.
    # ------------------------------------------------------------------
    if STEP_2_QUADRATIC_DAMPING:
        # Ng (2024) Table 4.1
        X_uu = 31.01
        Y_vv = 36.95
        K_pp = 2.08
        M_qq = 2.42
        N_rr = 2.42
        if STEP_3_PIECEWISE_HEAVE:
            # Use the directional values from Ng (2024)
            Z_ww = if_else(w < 0, 113.86, 76.69)
        else:
            # Use a single representative value when not piecewise.
            # The dive value (113.86) is more conservative and ensures
            # sufficient downward damping; the rise value is 76.69.
            # Average (95.28) is also reasonable — choose dive to be safe.
            Z_ww = 113.86
    else:
        # Baseline: no quadratic damping
        X_uu = 0.0
        Y_vv = 0.0
        Z_ww = 0.0
        K_pp = 0.0
        M_qq = 0.0
        N_rr = 0.0

    # ------------------------------------------------------------------
    # Effective inertia  =  rigid-body  +  added mass
    # M_z may be a CasADi expression when STEP_3 is active.
    # ------------------------------------------------------------------
    M_x     = m + X_udot
    M_y     = m + Y_vdot
    M_z     = m + Z_wdot
    M_phi   = Ix + K_pdot
    M_theta = Iy + M_qdot
    M_psi   = Iz + N_rdot

    # ==================================================================
    # THRUSTER ALLOCATION MATRIX  K  (6 × 8)   [ENU-FPU body frame]
    # Rows : [X, Y, Z, K, M, N]  (surge, sway, heave, roll, pitch, yaw)
    # Cols : thrusters 0–7
    # ==================================================================
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

    # ==================================================================
    # EQUATIONS OF MOTION  (Newton–Euler, body frame)
    #
    # IMPORTANT: velocity dynamics are named du, dv, dw, dp, dq, dr
    # to avoid overwriting the CasADi state symbols u, v, w, p, q, r.
    # Overwriting those symbols was a critical bug in the original code
    # that corrupted the kinematic equations (dx, dy, dz).
    # ==================================================================

    # --- Translational dynamics ---
    du = (  Kt[0]
          + m * r * v  -  m * q * w
          - buoyancy * sin(theta)
          - X_u  * u
          - X_uu * u * fabs(u)
         ) / M_x

    dv = (  Kt[1]
          - m * r * u  +  m * p * w
          + buoyancy * cos(theta) * sin(phi)
          - Y_v  * v
          - Y_vv * v * fabs(v)
         ) / M_y

    dw = (  Kt[2]
          + m * q * u  -  m * p * v
          + buoyancy * cos(theta) * cos(phi)
          - Z_w  * w
          - Z_ww * w * fabs(w)
         ) / M_z

    # --- Rotational dynamics ---
    dp = (  Kt[3]
          + (Iy - Iz) * q * r
          - m * ZG * g * cos(theta) * sin(phi)
          - K_p  * p
          - K_pp * p * fabs(p)
         ) / M_phi

    dq = (  Kt[4]
          + (Iz - Ix) * p * r
          - m * ZG * g * sin(theta)
          - M_q  * q
          - M_qq * q * fabs(q)
         ) / M_theta

    dr = (  Kt[5]
          - (Iy - Ix) * p * q
          - N_r  * r
          - N_rr * r * fabs(r)
         ) / M_psi

    # ==================================================================
    # KINEMATICS  (ZYX Tait-Bryan, body frame → world frame)
    # Standard rotation matrix J1(η2) from Fossen / Ng (2024) Eq. 2.10
    # ==================================================================
    dx = (  cos(psi)*cos(theta)                               ) * u \
       + ( -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi)  ) * v \
       + (  sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta)  ) * w

    dy = (  sin(psi)*cos(theta)                               ) * u \
       + (  cos(psi)*cos(phi) + sin(phi)*sin(theta)*sin(psi)  ) * v \
       + ( -cos(psi)*sin(phi) + sin(theta)*sin(psi)*cos(phi)  ) * w

    dz = ( -sin(theta)            ) * u \
       + (  cos(theta)*sin(phi)   ) * v \
       + (  cos(theta)*cos(phi)   ) * w

    # Angular kinematics: body rates → Euler angle rates  (J2 inverse, Ng Eq. 2.13)
    dphi   =  p  +  (sin(phi)*sin(theta)/cos(theta)) * q  +  (cos(phi)*sin(theta)/cos(theta)) * r
    dtheta =          cos(phi) * q  -  sin(phi) * r
    dpsi   =  (sin(phi)/cos(theta)) * q  +  (cos(phi)/cos(theta)) * r

    f_expl = vertcat(dx, dy, dz, dphi, dtheta, dpsi, du, dv, dw, dp, dq, dr)
    f_impl = sym_xdot - f_expl

    # ==================================================================
    # ACADOS MODEL ASSEMBLY
    # ==================================================================
    cost_y_expr = vertcat(sym_x, sym_u)

    model = AcadosModel()
    model.f_impl_expr   = f_impl
    model.f_expl_expr   = f_expl
    model.x             = sym_x
    model.xdot          = sym_xdot
    model.u             = sym_u
    model.cost_y_expr   = cost_y_expr
    model.cost_y_expr_e = sym_x
    model.name          = model_name

    return model