# Import libraries
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

from model import Cell, Sommet, Network, Simulation

# Exemple 1
# Liste de sommets
# vertices = {"T1":["X1"], "X1":["X2"], "X2":["X3"], "X3":[]} # dictionnaire {nom_sommet: [voisins]}
# v_properties = {"T1":[100], "X1":[500, 70/3.6, 1000/3600, 20/3.6, 0], "X2":[500, 70/3.6, 1000/3600, 20/3.6, 0], "X3":[500, 70/3.6, 1000/3600, 20/3.6, 0]} # for the tanks the only parameter is x_0
# Exemple 2
vertices = {"T1":["X1"], "T2":["X4"], "X1":["X2"], "X2":["X3"], "X3":["X4"], "X4":["X5"], "X5":["X6"], "X6":[]} # dictionnaire {nom_sommet: [voisins]}
v_properties = {"T1":[100,0], "T2":[100,0], 
                "X1":[200, 70/3.6, 1000/3600, 20/3.6, 0, 0], "X2":[200, 70/3.6, 1000/3600, 20/3.6, 0, 0],\
                 "X3":[200, 70/3.6, 1000/3600, 20/3.6, 0, 0], "X4":[200, 70/3.6, 1000/3600, 20/3.6, 0, 0],\
                   "X5":[200, 70/3.6, 1000/3600, 20/3.6, 0, 0], "X6":[200, 70/3.6, 1000/3600, 20/3.6, 0, 0]} # for the tanks the only parameter is x_0


net = Network(vertices)
graph, sommets = net.create_graph()
simu = Simulation(graph,sommets)
# commentaire test
# Exemple 1
# Définition des positions manuelles
# positions = {
#     sommets["T1"]: (1, 1),
#     sommets["X1"]: (1, 0),
#     sommets["X2"]: (2, 0),
#     sommets["X3"]: (3, 0)
# }

# Exemple 2
# Définition des positions manuelles
positions = {
    sommets["T1"]: (1, 1),
    sommets["T2"]: (4, 1),
    sommets["X1"]: (1, 0),
    sommets["X2"]: (2, 0),
    sommets["X3"]: (3, 0),
    sommets["X4"]: (4, 0),
    sommets["X5"]: (5, 0),
    sommets["X6"]: (6, 0)
}

print(graph.nodes)
print(graph.edges)

####################  To visualize the graph only ###################
# nx.draw(
#     graph,
#     pos=positions,
#     with_labels=True,
#     node_color="white",     # remplissage blanc
#     edgecolors="black",     # bordure noire
#     node_size=800,          # taille des sommets
#     linewidths=1.5          # épaisseur de la bordure
# )
# plt.show()

####################  To simulate the system and plot results ###################
# simu.setting(v_properties)
# simu.simu(0.1,10000,False)
# simu.results_2(0.1,10000)

#### To plot both strategies on the same graph
# N_simu = 10000
# h_simu = 0.1

# simu2 = Simulation(graph,sommets)

# simu.setting(v_properties)
# simu.simu(h_simu,N_simu,False)

# simu2.setting(v_properties)
# simu2.simu(h_simu,N_simu,True)

# # on récupère les données de chaque simulation et on les affiche ensemble
# T1,X1 = simu.T,simu.X
# T2,X2 = simu2.T,simu2.X

# first_key = next(iter(T1))
# print(len(T1[first_key].x))

# t_inputs = np.arange(N_simu)*h_simu
# t_states = np.arange(N_simu)*h_simu

# fig_T,axs_T = simu.create_subplot(T1,t_inputs,4,'Simultaneous evacuation')
# simu.add_data(fig_T,axs_T,t_inputs,T2,"Delayed evacuation")
# plt.show()

# fig_X,axs_X = simu.create_subplot(X1,t_states,6,'Simultaneous evacuation')
# simu.add_data(fig_X,axs_X,t_states,X2,"Delayed evacuation")
# plt.show()

######################  Verification of simulator  ##########################



# Number of iterations
N = 2000
# MPC horizon
nh = int(N/10)
M = int(nh/2)
# Nb of tanks
nt = 2
# Nb of cells
nc = 6
# Step time
h = 0.1
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

simu.setting(v_properties)
X_hist, U_hist = simu.verif_simu(h, M, N, nh, nt, nc, V, W, L, Cap, Fmax, Mt, Mc)
# simu.results_2_mpc(h, N, U_hist, X_hist)

# on récupère les données de chaque simulation et on les affiche ensemble
T1,X1 = simu.T,simu.X

first_key = next(iter(T1))
print("Taille dico" + str(len(T1[first_key].x)))
print("Taille X_hist" + str(len(X_hist)))

t_inputs = np.arange(N+1)*h
t_states = np.arange(N+1)*h

fig_T,axs_T = simu.create_subplot(T1,t_inputs,4,"Simulator")
simu.add_data_list(fig_T,axs_T,t_inputs,X_hist[0:2,:],"MPC")
plt.show()

fig_X,axs_X = simu.create_subplot(X1,t_states,6,"Simulator")
simu.add_data_list(fig_X,axs_X,t_states,X_hist[2:,:],"MPC")
plt.show()
