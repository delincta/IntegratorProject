import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import cvxpy as cp

# ============================================================
# 1. Définition du Crazy Flipper (le système le plus drôle du monde)
# ============================================================
n = 4
m = 2
Ts = 0.02

# Système continu
A_c = np.array([[0.0,  0.0,  1.0,  0.0],
                [0.0,  0.0,  0.0,  1.0],
                [0.0,  0.0, -0.2,  0.0],
                [0.0,  0.0,  0.0, -0.2]])

B_c = np.array([[0.0,   0.0],
                [0.0,   0.0],
                [15.0, -15.0],   # flipper gauche pousse à droite, droit à gauche
                [20.0,  20.0]])  # les deux poussent vers le haut

# Discrétisation
from scipy.signal import cont2discrete
A, B, _, _,  = cont2discrete((A_c, B_c, np.eye(n), np.zeros((n,m))), Ts, method='zoh')[:4]

# Poids du MPC 
Q  = np.diag([100.0, 100.0, 1.0, 1.0])
Qf = np.diag([600.0, 600.0, 15.0, 15.0])
R  = np.diag([0.01, 0.01])
u_max = 1.0
S = 12.0          # on autorise des coups de flipper brutaux
T = 28            # horizon un peu long → loopings parfaits

# ============================================================
# 2. Fonction MPC (version compacte avec warm-start)
# ============================================================
def solve_mpc(x0):
    X = cp.Variable((T+2, n))
    U = cp.Variable((T+1, m))

    cost = sum(cp.quad_form(X[t], Q) + cp.quad_form(U[t], R) for t in range(T+1))
    cost += cp.quad_form(X[T+1], Qf)

    constraints = [X[0] == x0]
    for t in range(T+1):
        constraints += [X[t+1] == A @ X[t] + B @ U[t]]
        constraints += [cp.norm_inf(U[t]) <= u_max]
    for t in range(T):
        constraints += [cp.norm_inf(U[t+1] - U[t]) <= S]

    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, warm_start=True, eps_abs=1e-4, eps_rel=1e-4)
    
    return U[0].value if prob.status == cp.OPTIMAL else np.zeros(m)

# ============================================================
# 3. Simulation en boucle fermée
# ============================================================
N_sim = 300                     # 300 × 0.02 s = 6 secondes de pure folie
x_traj = np.zeros((N_sim+1, n))
u_traj = np.zeros((N_sim, m))
x_traj[0] = [0.9, -0.8, 4.0, 6.0]   # la bille arrive en mode missile

for k in range(N_sim):
    u = solve_mpc(x_traj[k])
    if u is None:
        u = np.zeros(m)
    u_traj[k] = u
    x_traj[k+1] = A @ x_traj[k] + B @ u

# ============================================================
# 4. Animation qui déchire
# ============================================================
fig, ax = plt.subplots(figsize=(8,8))
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_title("Crazy Flipper MPC – regarde la bille faire des loopings puis se poser comme une fleur", fontsize=14)

# La bille
ball, = ax.plot([], [], 'o', color="#FF2F00", markersize=18, markeredgecolor='k', lw=2)

# Trajectoire
trail, = ax.plot([], [], color='#00FFFFFF', alpha=0.8, lw=2)

# Flipper gauche et droit (représentés par des barres)
left_flipper = ax.plot([], [], color='#00FF00', lw=8, alpha=0.7)[0]
right_flipper = ax.plot([], [], color='#00FF00', lw=8, alpha=0.7)[0]

x_hist = []

def init():
    ball.set_data([], [])
    trail.set_data([], [])
    left_flipper.set_data([], [])
    right_flipper.set_data([], [])
    return ball, trail, left_flipper, right_flipper

def update(frame):
    x_hist.append(x_traj[frame][:2].copy())
    if len(x_hist) > 80: 
        x_hist.pop(0)
    
    hist = np.array(x_hist)
    trail.set_data(hist[:,0], hist[:,1])
    
    ball.set_data([x_traj[frame,0]], [x_traj[frame,1]])
    
    # Visualisation des flippers (simplifiée)
    intensity = np.clip(np.abs(u_traj[frame]), 0, 1)
    left_flipper.set_data([-1.4, -1.2], [-1.3, -1.3 + intensity[0]])
    right_flipper.set_data([ 1.4,  1.2], [-1.3, -1.3 + intensity[1]])
    
    return ball, trail, left_flipper, right_flipper

ani = FuncAnimation(fig, update, frames=N_sim, init_func=init, blit=True, interval=20, repeat=False)

plt.tight_layout()
plt.show()

# Bonus : sauvegarde possible en .gif (décommente si tu veux)
# ani.save('crazy_flipper_mpc.gif', writer='pillow', fps=50)