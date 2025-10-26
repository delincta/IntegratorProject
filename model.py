import networkx as nx
import matplotlib.pyplot as plt

class Cell:
    def __init__(self, name, l, v, fmax, w, x_0):
        self.name = name
        self.l = l # length of road
        self.v = v # car speed
        self.fmax = fmax # max flow
        self.w = w # slope of supply function
        self.c = l/4.7 # average car length seems to be 4.7m, and we consider a single road
        self.x = [x_0] # trafic volume

    # demand function
    def d_fcn(self):
        return (min(self.v/self.l*self.x[-1],self.c))
    # supply function
    def s_fcn(self):
        return (max(self.c - self.w*self.x[-1],0))
    
class Sommet:
    def __init__(self, name):
        self.name = name
        

    def __repr__(self):
        return f"{self.name}"

class Network:
    def __init__(self, vertices):
        self.vertices = vertices  # dictionnaire {nom_sommet: [voisins]}

    def create_graph(self):
        G = nx.DiGraph()

        # Créer les sommets comme objets Sommet
        sommets_objets = {name: Sommet(name) for name in self.vertices}

        # Ajouter les sommets au graphe
        for sommet in sommets_objets.values():
            G.add_node(sommet)

        # Ajouter les arêtes
        for nom, voisins in self.vertices.items():
            for voisin in voisins:
                G.add_edge(sommets_objets[nom], sommets_objets[voisin])

        return G, sommets_objets
    
class Simulation:
    def __init__(self, graph, sommets):
        self.graph = graph
        self.sommets = sommets
        self.T = {}
        self.X = {}

    def f(self,i,j):
        if i.startswith("T"):
            return (min(float("inf"),self.X[j].s_fcn()))
        else:
            return (min(self.X[i].d_fcn(),self.X[j].s_fcn()))
    
    def f_end(self,i):
        return (self.X[i].d_fcn())
    
    def f_tank(self,i,s,u):
        return min(u*self.T[i].x[-1], self.X[s].s_fcn())
    

    # Paramètres simulation: h = disc. step, N = nb of steps
    def setting(self, v_properties):
        self.T = {}
        self.X = {}
        for i in self.graph.nodes():
            if i.name.startswith("T"):
                print(f"{i.name} is a tank")
                # print(v_properties[i.nom])
                self.T[i.name] = Cell(i.name,0,0,0,0,v_properties[i.name][0])
                print(v_properties[i.name][0])
            else:
                print(f"{i.name} is a portion of road")
                self.X[i.name] = Cell(i.name,*v_properties[i.name])
        # print(self.T)
        # print(self.X)
        return self.T, self.X
        
    def simu(self, h, N, u):
        # i = self.X["X1"]
        # l = list(self.graph.predecessors(self.sommets[i.name]))
        # print("Prédécesseurs de " + str(i.name) + ": ", l)
        # d = list(self.graph.successors(self.sommets[i.name]))
        # print("Successeurs de " + str(i.name) + ": ", d)
        # for z in d:
        #     # print(type(z.nom))
        #     print(self.X[z.name])
        for k in range (1,N):
            for t in self.T:
                d = list(self.graph.successors(self.sommets[t]))
                self.T[t].x.append(self.T[t].x[-1] - sum(h*(self.f_tank(t,s.name,u)) for s in d)) 
            for i in self.X:
                l = list(self.graph.predecessors(self.sommets[i]))
                # print("Voisins de " + str(i.name) + ": ", l)
                d = list(self.graph.successors(self.sommets[i]))
                if len(d) != 0:
                    somme = 0
                    for j in l:
                        if j.name.startswith("T"):
                            somme += self.f_tank(j.name,i,u)
                        else:
                            somme += self.f(j.name,i)

                    self.X[i].x.append(self.X[i].x[-1] + h*somme - sum(self.f(i,j.name) for j in d)) # ordre important
                else:
                    self.X[i].x.append(self.X[i].x[-1] + h*(sum(self.f(j.name,i) for j in l) - self.f_end(i))) # ordre important
                
    def results(self, h, N):
        t = []
        for k in range(N):
            t.append(k*h)
        for i in self.T:
            plt.figure
            plt.plot(t,self.T[i].x)
            plt.title(i)
            plt.show()
        for i in self.X:
            plt.figure
            plt.plot(t,self.X[i].x)
            plt.title(i)
            plt.show()

                


