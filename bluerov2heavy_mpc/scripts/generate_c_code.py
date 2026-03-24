from acados_template import AcadosOcp, AcadosOcpSolver
from bluerov2heavy import export_bluerov2heavy_model
import numpy as np
import casadi
#from utils import plot_pendulum
import math
from scipy.linalg import block_diag

def main():
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()

    # set model
    model = export_bluerov2heavy_model()
    ocp.model = model

    Tf = 5
    
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu  # y includes x and u
    
    N = 100

    # set dimensions
    ocp.dims.N = N

    # set cost
    W_x = np.diag([300, 300, 300, 100, 100, 150, 150, 150, 150, 50, 50, 50])    #Q_mat
    W_u = np.diag([1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0])                  #R_mat
    # the old R cost:
    W_u = np.diag([0.2, 0.2, 0.2, 0.2, 0.4, 0.4, 0.4, 0.4])                  #R_mat

    W = block_diag(W_x, W_u)
    ocp.cost.W_e = W_x
    ocp.cost.W = W

    # the 'EXTERNAL' cost type can be used to define general cost terms
    # NOTE: This leads to additional (exact) hessian contributions when using GAUSS_NEWTON hessian.
    ocp.cost.cost_type = 'NONLINEAR_LS'                 # weights times states (nonlinear relationship)
    ocp.cost.cost_type_e = 'NONLINEAR_LS'               # end states cost
    
    # Optimization costs
    ocp.cost.Vx = np.zeros((ny, nx))                    # map state to cost
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)                  # weight x
    ocp.cost.Vx_e = np.eye(nx)                          # end x cost
    ocp.cost.Vu = np.zeros((ny, nu))                    # map control to cost
    ocp.cost.Vu[-nu:, :] = np.eye(nu)                   # weight u
    

    # set constraints
    u_min = np.array([-40, -40, -40, -40, -40, -40, -40, -40])
    u_max = np.array([40, 40, 40, 40, 40, 40, 40, 40])
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4, 5, 6, 7])

    ocp.constraints.x0 = np.array([0.0, 0.0, -5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Angle constraints for roll and pitch
    ocp.constraints.idxbx = np.array([3, 4])  # indices of bounds on x (roll, pitch)
    ocp.constraints.lbx = np.array([-np.pi/3, -np.pi/3])
    ocp.constraints.ubx = np.array([np.pi/3, np.pi/3])
    
    # reference trajectory (will be overwritten later)
    x_ref = np.zeros(nx)
    u_ref = np.zeros(nu)
    ocp.cost.yref = np.concatenate((x_ref, u_ref))  # reference for y, which is [x;u]
    ocp.cost.yref_e = x_ref

    # set options
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
    # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
    ocp.solver_options.integrator_type = 'ERK'
    #ocp.solver_options.qp_solver_cond_N = 5
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI, SQP
    ocp.solver_options.qp_solver_iter_max = 100

    # set prediction horizon
    ocp.solver_options.tf = Tf

    ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

    simX = np.ndarray((N+1, nx))
    simU = np.ndarray((N, nu))

    status = ocp_solver.solve()
    ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")

    if status != 0:
        raise Exception(f'acados returned status {status}.')

    # get solution
    for i in range(N):
        simX[i,:] = ocp_solver.get(i, "x")
        simU[i,:] = ocp_solver.get(i, "u")
    simX[N,:] = ocp_solver.get(N, "x")

    #plot_pendulum(np.linspace(0, Tf, N+1), Fmax, simU, simX, latexify=False)


if __name__ == '__main__':
    main()