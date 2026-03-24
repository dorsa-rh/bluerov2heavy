"""
generate_c_code_ampc.py
========================
Generates the ACADOS C solver for the BlueROV2 Heavy AMPC
(Adaptive MPC with EAOB + RLS-VFF online system identification).

Run once before catkin_make:
    cd ~/catkin_ws/src/bluerov2/bluerov2heavy_dobmpc/scripts
    python3 generate_c_code_ampc.py

Solver name: bluerov2heavy  (same as DOB — only one solver name is used;
  AMPC and DOB share the solver but differ in how params are set at runtime)

NP = 24  (6 disturbances + 6 added_mass + 6 lin_damp + 6 nonlin_damp)
NU = 8
NX = 12
N  = 100
Tf = 5.0 s

PARAMETER LAYOUT (must match bluerov2heavy_ampc.cpp solve()):
  [0-5]   : environmental disturbances  Xw, Yw, Zw, Kw, Mw, Nw
  [6-11]  : added mass                  Xu', Yv', Zw', Kp', Mq', Nr'
  [12-17] : linear damping              X_u, Y_v, Z_w, K_p, M_q, N_r
  [18-23] : nonlinear damping           X_uu, Y_vv, Z_ww, K_pp, M_qq, N_rr
"""

import sys

# ── Tera compatibility fix ────────────────────────────────────────────────────
try:
    from acados_template import get_tera
    print("Setting up tera 0.0.34...")
    get_tera(tera_version='0.0.34', force_download=True)
    print("✓ Tera ready.")
except Exception as e:
    print(f"Warning: tera setup issue: {e}\nContinuing anyway...")

from acados_template import AcadosOcp, AcadosOcpSolver
from bluerov2heavy_ampc_model import export_bluerov2heavy_ampc_model
import numpy as np
from scipy.linalg import block_diag


def main():
    ocp = AcadosOcp()

    # ── Model ─────────────────────────────────────────────────────────────────
    model = export_bluerov2heavy_ampc_model()
    ocp.model = model

    nx     = model.x.size()[0]    # 12
    nu     = model.u.size()[0]    # 8
    nparam = model.p.size()[0]    # 24
    ny     = nx + nu              # 20

    N  = 100
    Tf = 5.0

    ocp.dims.N = N
    ocp.solver_options.tf = Tf

    # ── Runtime parameter initialisation  (Ng 2024 nominal values) ───────────
    # These are the values used when COMPENSATE_D = false.
    # When COMPENSATE_D = true, all 24 are overwritten by RLS estimates.
    p0 = np.zeros(nparam)
    # [0-5]  disturbances: start at zero
    # [6-11] added mass: Ng 2024 nominal
    p0[6]  = 10.77   # Xu'
    p0[7]  = 24.86   # Yv'
    p0[8]  = 34.60   # Zw'  (dive nominal)
    p0[9]  = 0.103   # Kp'
    p0[10] = 0.120   # Mq'
    p0[11] = 0.120   # Nr'
    # [12-17] linear damping (negative = opposing motion)
    p0[12] = -38.95  # X_u
    p0[13] = -60.11  # Y_v
    p0[14] = -49.50  # Z_w
    p0[15] =   0.0   # K_p  (experimentally zero)
    p0[16] =   0.0   # M_q
    p0[17] =   0.0   # N_r
    # [18-23] nonlinear damping
    p0[18] = -31.01  # X_uu
    p0[19] = -36.95  # Y_vv
    p0[20] = -113.86 # Z_ww
    p0[21] =  -2.08  # K_pp
    p0[22] =  -2.42  # M_qq
    p0[23] =  -2.42  # N_rr

    ocp.parameter_values = p0

    # ── Cost  (same structure as DOB solver) ─────────────────────────────────
    W_x = np.diag([300, 300, 300,
                   100, 100, 150,
                   150, 150, 150,
                    50,  50,  50])

    W_u = np.diag([0.2, 0.2, 0.2, 0.2,   # horizontal thrusters
                   0.4, 0.4, 0.4, 0.4])   # vertical thrusters

    W = block_diag(W_x, W_u)
    ocp.cost.W   = W
    ocp.cost.W_e = W_x

    ocp.cost.cost_type   = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    ocp.cost.Vx = np.zeros((ny, nx));  ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu));  ocp.cost.Vu[-nu:, :]  = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref   = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(nx)

    # ── Input constraints  ±40 N per thruster ────────────────────────────────
    ocp.constraints.lbu   = -40.0 * np.ones(nu)
    ocp.constraints.ubu   =  40.0 * np.ones(nu)
    ocp.constraints.idxbu = np.arange(nu)

    # ── State constraints: roll and pitch ±60° ────────────────────────────────
    ocp.constraints.idxbx = np.array([3, 4])
    ocp.constraints.lbx   = np.array([-np.pi/3, -np.pi/3])
    ocp.constraints.ubx   = np.array([ np.pi/3,  np.pi/3])

    # ── Initial state ─────────────────────────────────────────────────────────
    x0 = np.zeros(nx)
    x0[2] = -5.0
    ocp.constraints.x0 = x0

    # ── Solver options ────────────────────────────────────────────────────────
    ocp.solver_options.qp_solver           = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx      = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type     = 'ERK'
    ocp.solver_options.nlp_solver_type     = 'SQP_RTI'
    ocp.solver_options.qp_solver_iter_max  = 100
    ocp.solver_options.print_level         = 0

    # ── Generate C code ───────────────────────────────────────────────────────
    ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp_ampc_heavy.json')

    status = ocp_solver.solve()
    ocp_solver.print_statistics()
    if status != 0:
        print(f"Warning: initial solve returned status {status}.")
    else:
        print("✓ AMPC solver generated and verified successfully.")

    # Print constants for the header file
    print(f"\n── Header constants ──────────────────────────────")
    print(f"#define BLUEROV2HEAVY_NX  {nx}")
    print(f"#define BLUEROV2HEAVY_NU  {nu}")
    print(f"#define BLUEROV2HEAVY_NP  {nparam}   // AMPC: 24 params")
    print(f"#define BLUEROV2HEAVY_N   {N}")
    print(f"#define BLUEROV2HEAVY_NY  {ny}")
    print(f"──────────────────────────────────────────────────\n")


if __name__ == '__main__':
    main()
    