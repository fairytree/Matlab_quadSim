import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits as mpltk


def V(x, v, P):

    """
    Calculates the Lyapunov value.

    Args:
        x: current state
        v: current applied reference
        P: matrix used for calculation
    """

    return (x - xHatV(v)).T @ P @ (x - xHatV(v))

def P(K_P, epsilon, K_D):

    """
    Gets the matrix $P$ for calculating the Lyapunov threshold value.
    """

    return 1 / 2 * np.block([[K_P + epsilon * K_D**2, epsilon * K_D],
                             [epsilon * K_D,          np.eye(3)]])

def gamma(v, d, c, P):

    r"""
    Gets the Lyapunov threshold value, $\Gamma(v)$ from the applied reference $v$. The constraint is $c^\top x\leq d(v)$, and the Lyapunov function is $V(x, v) = (x - \hat{x}_v)^\top P (x - \hat{x}_v)$ (with $P > 0$).

    Args:
        v: current applied reference
        d: part of the constraint
        c: part of the constraint
        P: used for calculating Lyapunov value
    """

    return (-c.T @ xHatV(v) + d)**2 / (c.T @ np.linalg.inv(P) @ c)

def xHatV(v):
    
    """
    Gets the equilibrium state $\hat{x}_v$.

    Args:
        v: applied reference
    """

    return np.concatenate([v, np.array([0.0, 0.0, 0.0])])

def DSMs(T_max, T_min, m, g, k_P, k_D, epsilon, x, v, P, kappa_s):

    """
    Gets the dynamic safety margin of the saturation constraint, $\Delta^s_i$.

    Args:
        T_max: maximum thrust
        T_min: minimum thrust
        m: mass of quadrotor
        g: gravitational acceleration
        k_P: proportional gain for outer loop controller
        k_D: derivative gain for outer loop controller
        epsilon: a constant between 0 and 1
        x: current state
        v: current applied reference
        P: used for calculating the Lyapunov threshold value
        kappa_s: a positive scaling factor
    """

    # threshold value is computed seperately for T_min and T_max
    gamma_T_max = (T_max - m * g)**2 * (k_P + epsilon * (1 - epsilon) * k_D**2) / (2 * m**2 * (k_P**2 + k_D**2 * (k_P + epsilon * k_D**2 - 2 * epsilon * k_P)))
    gamma_T_min = (T_min - m * g)**2 * (k_P + epsilon * (1 - epsilon) * k_D**2) / (2 * m**2 * (k_P**2 + k_D**2 * (k_P + epsilon * k_D**2 - 2 * epsilon * k_P)))

    return kappa_s * (np.min([gamma_T_max, gamma_T_min]) - V(x, v, P))

def DSMo(obstacle_pos_ls, x, v, P, R_o_ls, R_a, kappa_o):

    """
    Gets the dynamic safety margin of the obstacle constraint, $\Delta^o_i$.
    """

    # list of Lyapunov threshold values for each of the obstacles
    gamma_o_ls = []
    
    # loop over all the obstacles and compute their Lypunov threshold values
    for i in range(np.shape(obstacle_pos_ls)[0]):

        c = np.concatenate([v - obstacle_pos_ls[i], np.array([0, 0, 0])])
        c = c / np.linalg.norm(c)

        d = c[0:3].T @ v - R_o_ls[i] - R_a - np.linalg.norm(v - obstacle_pos_ls[i])

        gamma_o_ls.append(gamma(v, d, c, P))

    return kappa_o * (np.min(gamma_o_ls) - V(x, v, P))

def DSM(DSM_ls):

    """
    Gets the dynamic saftety margin.

    Args:
        DSM_ls: list of dynamic saftey margins for different constraints (e.g. saturation and obstacle constraints)
    """

    return np.max(np.min(DSM_ls), 0)

def main():
    
    # example variables (all example variables have a suffix of _ex)
    kappa_s_ex = 2.5
    kappa_o_ex = 20 # the paper use 20
    epsilon_ex = 0.5 # epsilon is between 0 and 1
    k_P_ex = 13
    k_D_ex = 5
    K_P_ex = k_P_ex * np.eye(3)
    K_D_ex = k_D_ex * np.eye(3)
    x_ex = np.array([0, 0, 0, 0, 0, 0])
    v_ex = np.array([0.1, 0.1, 0.1])
    obstacle_pos_ls_ex = np.array([[0.0, 0.0, 0.0]])
    # obstacle_pos_ls_ex = np.array([[0.4, 0.5, 0.25], [100, 100, 100]])
    T_max_ex = 0.59 # unit: N
    T_min_ex = 0.0
    m_ex = 0.0346 # unit: kg
    g_ex = 9.81
    R_o_ls_ex = 0.2 * np.ones(np.size(obstacle_pos_ls_ex)) # unit: meter
    R_a_ex = 0.08 # unit: meter
    J_ex =  10**-6 * np.array([[17.31, 0, 0], [0, 17.94, 0], [0, 0, 33.75]]) # unit: kg*m^2
    eta_ex = 0.005
    P_ex = P(K_P_ex, epsilon_ex, K_D_ex)
    DSM_s_ex = DSMs(T_max_ex, T_min_ex, m_ex, g_ex, k_P_ex, k_D_ex, epsilon_ex, x_ex, v_ex, P_ex, kappa_s_ex) # s means saturation
    DSM_o_ex = DSMo(obstacle_pos_ls_ex, x_ex, v_ex, P_ex, R_o_ls_ex, R_a_ex, kappa_o_ex) # o stands for obstacle
    DSM_ex = DSM([DSM_o_ex, DSM_s_ex])
    
    # print out results
    print(f"DSM for obstacles: {DSM_o_ex}")
    print(f"DSM for saturation: {DSM_s_ex}")
    print(f"DSM: {DSM_ex}")

    # visualize

    # initialize the figure
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.set_zlim([0, 2])

    # create meshgrid
    x = np.linspace(-0.3, 0.3, 1000)
    y = np.linspace(-0.3, 0.3, 1000)
    z = np.linspace(-0.3, 0.3, 1000)
    [x, y] = np.meshgrid(x, y)
    # [x, z] = np.meshgrid(x, z)

    # initialize matrix of Lyapunov threshold values
    x_shape = np.shape(x)
    C = np.zeros([x_shape[0], x_shape[1]])

    # loop over every point in the meshgrid and compute their Lyapunov threshold value
    for i in range(x_shape[0]):
        
        for j in range(x_shape[1]):
            
            # obstacle_pos = [[x[i, j], y[i, j], 0]]
            v = np.array([x[i, j], y[i, j], 0])
            # C[i, j] = DSMo(obstacle_pos_ls_ex, x_ex, v, P_ex, R_o_ls_ex, R_a_ex, kappa_o_ex)
            
            gamma_o_ls = []
            
            for k in range(len(obstacle_pos_ls_ex)):
                
                c = np.concatenate([v - obstacle_pos_ls_ex[k], np.array([0, 0, 0])])
                c = c / np.linalg.norm(c)
                d = c[0:3].T @ v - R_o_ls_ex[k] - R_a_ex - np.linalg.norm(v - obstacle_pos_ls_ex[k])
                gamma_o_ls.append(gamma(v, d, c, P_ex))
            
            C[i, j] = min(gamma_o_ls)

    # plot the surface and show its colorbar
    surface = ax.plot_surface(x, y, C, linewidth=0)
    # surface = ax.plot_surface(x, z, C, linewidth=0)
    fig.colorbar(surface)

    # show the figure
    plt.show()


if __name__ == "__main__":

    main()
