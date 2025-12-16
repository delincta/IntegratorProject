# Import libraries
import numpy as np
np.set_printoptions(precision=2, suppress=True)
import matplotlib.pyplot as plt
from cvxpy import *


########## Data ##############
# Number of iterations
N = 2000
# MPC horizon
# nh = 30
nh = int(N/10)
M = int(nh/2)
# Nb of tanks
nt = 2
# Nb of cells
nc = 6
# Step time
h = 0.1
x_max = 100 #nombre de véhicules max
# Max speeds of the cells
V = np.array([70, 70, 70, 70, 70, 70], dtype=float).reshape(nc,1)
# Slopes of supply function
W = 20/3.6*np.eye(6)
# W = np.array([ 20/3.6,  20/3.6,  20/3.6,  20/3.6,  20/3.6,  20/3.6], dtype=float).reshape(nc,1)
# Lengths of roads
L = np.array([500, 500, 500, 500, 500, 500], dtype=float).reshape(nc,1)
# Capacities of the cells
Cap = 1/4.7*L
# Max flow
Fmax = np.array([1000/3600, 1000/3600, 1000/3600, 1000/3600, 1000/3600, 1000/3600], dtype=float).reshape(nc,1)

Mt = np.array([[-1, 0],
               [0, -1], 
               [1, 0],
               [0, 0],
               [0, 0],
               [0, 1],
               [0, 0],
               [0, 0]], dtype=float)

Mc = np.array([[0, 0, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0], 
               [-1, 0, 0, 0, 0, 0],
               [1, -1, 0, 0, 0, 0],
               [0, 1, -1, 0, 0, 0],
               [0, 0, 1, -1, 0, 0],
               [0, 0, 0, 1, -1, 0],
               [0, 0, 0, 0, 1, -1]], dtype=float)

# Variables to store the values of flow, respectively for tanks and cells
Ft = Variable((nt, nh))
Fc = Variable((nc, nh))
# Variables to store values of supply function and demand function
Sf = Variable((nc, nh))
Df = Variable((nc, nh))
# Initial nb of vehicles in the tanks and in the cells
Xt0 = np.array([100, 100], dtype=float).reshape(-1,1)
Xc0 = np.array([0, 0, 0, 0, 0, 0], dtype=float).reshape(-1,1)
# Final nb of vehicles
Xtf = np.array([0, 0], dtype=float).reshape(-1,1)
Xcf = np.array([0, 0, 0, 0, 0, 0], dtype=float).reshape(-1,1)
# print("Xt0 = {}".format(Xt0))
# print("Xc0 = {}".format(Xc0))
# Constants converted in 1D
Xt0_1d = Constant(Xt0)   # shape (2,)
Xc0_1d = Constant(Xc0)  # shape (6,)

X0 = vstack([Xt0_1d, Xc0_1d])                 # shape (8,)
print(X0.shape)
# (optionnel si tu veux une cible souple plus tard)
Xtf_1d = Constant(Xtf)   # shape (2,)
Xcf_1d = Constant(Xcf)   # shape (6,)
Xf = vstack([Xtf_1d, Xcf_1d])                  # shape (8,)

# Describe the convex problem
# Complete vectors Xt and Xc
Xt = Variable((nt, nh+1))
Xc = Variable((nc, nh+1))
X = Variable((nt+nc, nh+1))
Gamma = Variable((nc, nh))
Z = Variable((nc, nh))
# Complete vector U of commands
U = Variable((nt, nh))

# Objective function
# obj = Minimize(sum(Xt))
rho = 1.0      # poids terminal (à ajuster)

# Objective function penalizes the number of vehicles in the tanks but it doesn't care about the number of vehicles 
# in the cells => the goal is to empty the tanks the fastest possible
# obj = Minimize(
#     sum_squares(X[0:2,:]) +
#     rho * sum_squares(X[:, -1] - Xf))
# Now we can try to penalize the number of vehicles present in the whole network

obj = Minimize(sum(X))
X_hist = np.zeros((nt+nc, N+1))  # stockage de tous les états
X_hist[:, 0] = X0.value.flatten(order='C')
print(X_hist[:, 0].shape)
U_hist = np.zeros((nt, N))    # stockage de toutes les commandes


for k_simu in range(0, N, M):
    print(k_simu)

    # Constraints
    constr = []
    # Dynamics
    # constr += [X[:, 0] == vstack((Xt0,Xc0)), X[:, 1:] == X[:,:N] + h*(Mt@Ft + Mc@Fc), X[:, -1] == vstack((Xtf,Xcf))]
    constr += [X[:, 0:1] == X0, X[:, 1:] == X[:,:nh] + h*(Mt@Ft + Mc@Fc)]
    for k in range(nh):
        constr += [Sf[:, k] <= (Cap - W @ X[2:8, k])]
        # constr += [Sf[:, k] >= 0]
        constr += [U[:, k] >= 0, U[:, k] <= X[0:2, k]]
        constr += [Ft[:, k] <= vstack([Sf[0, k], Sf[3, k]])]
        for i in range(nc):
            constr += [Z[i, k] <= x_max * Gamma[i, k],
                    Z[i, k] <= X[2+i, k]] # Gamma * X
            constr += [Df[i,k] <= Z[i, k] * V[i]/L[i]]
        constr += [Fc[5, k] == Df[5, k]]
        constr += [Fc[0:5, k] <= Sf[1:6, k]]
        constr += [Fc[0:5, k] <= Df[0:5, k]]

    constr += [Df <= Fmax]
    constr += [Ft >= 0]
    constr += [Fc >= 0]
    constr += [Ft <= U]
    constr += [Gamma >= 0, Gamma <= 1]
    prob = Problem(obj, constr)

    prob.solve(warm_start=True)

    u = U.value[:,0:M]
    
    U_hist[:, k_simu:k_simu+M] = u
    X_hist[:, k_simu+1:k_simu+M+1] = X.value[:,0:M]
    # print(X0)
    X0 = X.value[:,M-1].reshape(-1,1)
    # print(X0) 
       

# t = 0
# while t < N:
#     prob.solve()


#     # Appliquer M pas ou jusqu'à la fin
#     for j in range(M):
#         if t >= N:
#             break
#         0 <= j < nh
#         u = U.value[:,j]
        
#         U_hist[:, t] = u
#         X_hist[:, t+1] = X.value[:,j]
        
#         t += 1

X_hist = X_hist
U_hist = U_hist
print(X_hist.shape)
# solve the problem
#prob.solve(solver=GUROBI)

#print(prob.solver_stats.solver_name)
#print("Problem Status: {}".format(prob.status))
#print("Optimal value x* = : {}".format(X.value))
# print("Optimal solution u* = : {}".format(U.value))
# plt.step(np.arange(N), np.ravel(U.value[1,:]))
# plt.xlabel('Time t')
# plt.ylabel('Command u1(t)')
# plt.step(np.arange(N), np.ravel(U.value[0,:]))
# plt.xlabel('Time t')
# plt.ylabel('Command u1(t)')
# plt.show()

####################### Plot the inputs ######################
cols = 2
rows = (nt + cols - 1) // cols  # arrondi vers le haut

fig, axs = plt.subplots(rows, cols, figsize=(10, 6))

# axs est une matrice → on la transforme en liste pour itérer facilement
axs = axs.flatten()

# Boucle sur chaque variable
for i in range(nt):
    axs[i].plot(np.arange(N), (U_hist[i,:]))
    axs[i].set_title(f"u{i+1}")

# Supprimer les subplots vides si n n'est pas multiple de cols
for j in range(i+1, len(axs)):
    fig.delaxes(axs[j])

plt.tight_layout()
plt.show()


####################### Plot the states ######################
cols = 2
rows = (nt+nc + cols - 1) // cols  # arrondi vers le haut

fig, axs = plt.subplots(rows, cols, figsize=(10, 6))

# axs est une matrice → on la transforme en liste pour itérer facilement
axs = axs.flatten()

# Boucle sur chaque variable
for i in range(nt+nc):
    axs[i].step(np.arange(N+1), np.ravel(X_hist[i,:]))
    if i < nt:
        axs[i].set_title(f"xt{i+1}")
    else:
        axs[i].set_title(f"x{i+1-2}")
    
# Supprimer les subplots vides si n n'est pas multiple de cols
for j in range(i+1, len(axs)):
    fig.delaxes(axs[j])

plt.tight_layout()
plt.show()