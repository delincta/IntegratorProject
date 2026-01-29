# Import libraries
import numpy as np
np.set_printoptions(precision=2, suppress=True)
import matplotlib.pyplot as plt
from cvxpy import *


########## Data ##############
# Number of iterations
N = 8000
# MPC horizon
nh = int(N/5)
M = int(nh/2)
# Nb of tanks
nt = 2
# Nb of cells
nc = 6
# Step time
h = 0.1
x_max = 100 # Max number of vehicles/ only used to find Gamma
# Max speeds of the cells
V = 1/3.6*np.array([70, 70, 70, 70, 70, 70], dtype=float).reshape(nc,1)
# Slopes of supply function
W = 20/3.6*np.eye(6)
# Lengths of roads
L = np.array([200, 200, 200, 200, 200, 200], dtype=float).reshape(nc,1)
# Capacities of the cells
Cap = 1/4.7*L *20/3.6 *1 # 1 voie mobilisée
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

# =========================
# Boucle MPC optimisée pour le temps d'évacuation
# =========================

# Initialisation
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

X0 = vstack([Xt0_1d, Xc0_1d])  # état initial
X_hist = np.zeros((nt+nc, N+1))  
X_hist[:, 0] = X0.value.flatten()
Sf_hist = np.zeros((nc, N))
Df_hist = np.zeros((nc, N)) 
Fc_hist = np.zeros((nc, N))

for k_simu in range(0, N, M):
    print(k_simu)

    # --- Variables pour l'horizon MPC ---
    Xt = Variable((nt, nh+1))
    Xc = Variable((nc, nh+1))
    X = Variable((nt+nc, nh+1))
    Ft = Variable((nt, nh))
    Fc = Variable((nc, nh))
    Sf = Variable((nc, nh))
    Df = Variable((nc, nh))
    
    # Cumulative vehicles exited
    Y = Variable(nh+1)

    # --- Contraintes ---
    constr = []

    # état initial
    constr += [X[:, 0:1] == X0]

    # dynamique
    for k in range(nh):
        constr += [X[:, k+1] == X[:, k] + h*(Mt@Ft[:,k] + Mc@Fc[:,k])]

        # demande et offre
        constr += [Df[:,k] <= Fmax]
        constr += [Sf[:,k] <= (Cap - W @ X[2:8, k])]
        constr += [Ft[:,k] <= vstack([Sf[0,k], Sf[3,k]])]

        for i in range(nc):
            constr += [Df[i,k] <= V[i]/L[i]*X[2+i, k]]

        # flux cellulaire
        constr += [Fc[5,k] == Df[5,k]]
        constr += [Fc[0:5,k] <= Sf[1:6, k]]
        constr += [Fc[0:5,k] <= Df[0:5, k]]

    # bornes
    constr += [Ft >= 0, Fc >= 0, X >= 0]

    # cumulative vehicles exited
    constr += [Y[0] == 0]
    for k in range(nh):
        constr += [Y[k+1] == Y[k] + h * Fc[5,k]]

    # --- Objectif : maximiser le flux sorti ---
    alpha = 1e-3  # petit poids pour éviter accumulation excessive
    obj = Maximize(Y[-1] - alpha*sum(X))

    # --- Résolution ---
    prob = Problem(obj, constr)
    prob.solve(warm_start=True)

    # --- Stockage des résultats ---
    X_hist[:, k_simu+1:k_simu+M+1] = X.value[:, 0:M]
    Sf_hist[:, k_simu:k_simu+M] = Sf.value[:,0:M]
    Df_hist[:, k_simu:k_simu+M] = Df.value[:,0:M]
    Fc_hist[:, k_simu:k_simu+M] = Fc.value[:,0:M]

    # mise à jour de l'état initial pour le prochain horizon
    X0 = X.value[:, M].reshape(-1,1)

# =========================
# Affichage des résultats
# =========================

###################### Print the cost #######################
cost = np.sum(X_hist)
print("Coût total: " + str(cost))

###################### Print Df and Fc #######################
cols = 2
rows = (nc + cols - 1) // cols  # arrondi vers le haut

fig, axs = plt.subplots(rows, cols, figsize=(10, 6))

# axs est une matrice → on la transforme en liste pour itérer facilement
axs = axs.flatten()

t_inputs = np.arange(N)*h
# Boucle sur chaque variable
# for i in range(nt):
#     axs[i].plot(t_inputs, (U_hist[i,:]))
#     axs[i].set_title(f"u{i+1}")
for i in range(nc):
    axs[i].plot(t_inputs, (Fc_hist[i,:]), '-', label='Fc')
    axs[i].plot(t_inputs, (Df_hist[i,:]), '--', label='Df')
    axs[i].set_title(f"Df/Fc{i+1}")
    axs[i].legend()

# Supprimer les subplots vides si n n'est pas multiple de cols
for j in range(i+1, len(axs)):
    fig.delaxes(axs[j])

plt.tight_layout()
plt.show()

###################### Print the states #######################
t_states = np.arange(N+1)*h
cols = 2
rows = (nt+nc + cols - 1) // cols
fig, axs = plt.subplots(rows, cols, figsize=(10,6))
axs = axs.flatten()

for i in range(nt+nc):
    axs[i].step(t_states, X_hist[i,:])
    if i < nt:
        axs[i].set_title(f"State of T{i+1}")
    else:
        axs[i].set_title(f"State of X{i+1-2}")
for j in range(i+1, len(axs)):
    fig.delaxes(axs[j])
plt.tight_layout()
plt.show()
