"""
generate_c_code_dob.py
=======================
Generates the ACADOS C solver for the BlueROV2 Heavy DOBMPC.

Run once before catkin_make:
    cd ~/catkin_ws/src/bluerov2/bluerov2heavy_dobmpc/scripts
    python3 generate_c_code_dob.py

Solver name: bluerov2heavy  (matches model_name in bluerov2heavy_dob_model.py)
NP = 6   (disturbances: Xw, Yw, Zw, Kw, Mw, Nw)
NU = 8   (direct thruster forces)
NX = 12  (state: pos + body vel)
N  = 100 (horizon steps)
Tf = 5.0 s

CHANGES vs standard generate_c_code.py:
  - Import bluerov2heavy_dob_model instead of bluerov2
  - nu = 8,  NP = 6,  N = 100,  Tf = 5.0
  - u bounds: 8 × ±40 N (individual thruster limits)
  - Roll/pitch angle constraints added (±π/3)
  - W_u: vertical thrusters (4-7) penalised 2× horizontal (0-3)
  - Initial depth z0 = -5 m (ENU)
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
from bluerov2heavy_dob_model import export_bluerov2heavy_dob_model
import numpy as np
from scipy.linalg import block_diag


def main():
    ocp = AcadosOcp()

    # ── Model ─────────────────────────────────────────────────────────────────
    model = export_bluerov2heavy_dob_model()
    ocp.model = model

    nx     = model.x.size()[0]    # 12
    nu     = model.u.size()[0]    # 8
    nparam = model.p.size()[0]    # 6
    ny     = nx + nu              # 20

    N  = 100
    Tf = 5.0

    ocp.dims.N = N
    ocp.solver_options.tf = Tf

    # ── Runtime parameter initialisation ──────────────────────────────────────
    # All disturbances start at zero; updated each step by EAOB
    ocp.parameter_values = np.zeros(nparam)

    # ── Cost  (NONLINEAR_LS: minimise ||[x; u] - yref||²_W) ─────────────────
    # Q: position states weighted highest, velocities lower
    W_x = np.diag([300, 300, 300,   # x, y, z
                   100, 100, 150,   # phi, theta, psi
                   150, 150, 150,   # u, v, w
                    50,  50,  50])  # p, q, r

    # R: vertical thrusters (4-7) penalised 2× horizontal (0-3)
    # to reflect their smaller roll/pitch authority per Newton
    W_u = np.diag([0.2, 0.2, 0.2, 0.2,   # horizontal
                   0.4, 0.4, 0.4, 0.4])   # vertical

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
    u_lim = 40.0
    ocp.constraints.lbu    = -u_lim * np.ones(nu)
    ocp.constraints.ubu    =  u_lim * np.ones(nu)
    ocp.constraints.idxbu  = np.arange(nu)

    # ── State constraints: roll and pitch ±60° ────────────────────────────────
    ocp.constraints.idxbx = np.array([3, 4])   # phi, theta indices in x
    ocp.constraints.lbx   = np.array([-np.pi/3, -np.pi/3])
    ocp.constraints.ubx   = np.array([ np.pi/3,  np.pi/3])

    # ── Initial state ─────────────────────────────────────────────────────────
    # z = -5 m (ENU, ~5 m depth); all velocities zero
    x0 = np.zeros(nx)
    x0[2] = -5.0
    ocp.constraints.x0 = x0

    # ── Solver options ────────────────────────────────────────────────────────
    ocp.solver_options.qp_solver        = 'FULL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx   = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type  = 'ERK'
    ocp.solver_options.nlp_solver_type  = 'SQP_RTI'
    ocp.solver_options.qp_solver_iter_max = 100
    ocp.solver_options.print_level      = 0

    # ── Generate C code ───────────────────────────────────────────────────────
    ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp_dob_heavy.json')

    # Quick solve to verify the generated code compiles and runs
    status = ocp_solver.solve()
    ocp_solver.print_statistics()
    if status != 0:
        print(f"Warning: initial solve returned status {status} "
              f"(may be fine for zero-initialised problem).")
    else:
        print("✓ DOB solver generated and verified successfully.")

    # Print key constants for the header file
    print(f"\n── Header constants ──────────────────────────────")
    print(f"#define BLUEROV2HEAVY_NX  {nx}")
    print(f"#define BLUEROV2HEAVY_NU  {nu}")
    print(f"#define BLUEROV2HEAVY_NP  {nparam}")
    print(f"#define BLUEROV2HEAVY_N   {N}")
    print(f"#define BLUEROV2HEAVY_NY  {ny}")
    print(f"──────────────────────────────────────────────────\n")


if __name__ == '__main__':
    main()