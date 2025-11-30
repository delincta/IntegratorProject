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


# nx.draw(graph, pos=positions, with_labels=True)
# plt.show()

simu.setting(v_properties)
simu.simu(0.01,100000*2)
simu.results_2(0.01,100000*2)