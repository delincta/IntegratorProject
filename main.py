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
v_properties = {"T1":[100,0], "T2":[100,0], "X1":[500, 70/3.6, 1000/3600, 20/3.6, 0, 0], "X2":[500, 70/3.6, 1000/3600, 20/3.6, 0, 0],\
                 "X3":[500, 70/3.6, 1000/3600, 20/3.6, 0, 0], "X4":[500, 70/3.6, 1000/3600, 20/3.6, 0, 0],\
                   "X5":[500, 70/3.6, 1000/3600, 20/3.6, 0, 0], "X6":[500, 70/3.6, 1000/3600, 20/3.6, 0, 0]} # for the tanks the only parameter is x_0


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
# nx.draw(graph, pos=positions, with_labels=True)
# plt.show()

####################  To simulate the system and plot results ###################
# simu.setting(v_properties)
# simu.simu(0.01,100000*2)
# simu.results_2(0.01,100000*2)


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
V = np.array([70, 70, 70, 70, 70, 70], dtype=float).reshape(nc,1)
# Slopes of supply function
W = 20/3.6*np.eye(6)
# Lengths of roads
L = np.array([500, 500, 500, 500, 500, 500], dtype=float).reshape(nc,1)
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
simu.results_2_mpc(h, N, U_hist, X_hist)