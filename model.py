import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import cvxpy as cp


class Cell:
    def __init__(self, name, l, v, fmax, w, x_0, uref):
        self.name = name
        self.l = l # length of road
        self.v = v # car speed
        self.fmax = fmax # max flow
        self.w = w # slope of supply function
        self.c = w*l/4.7 # average car length seems to be 4.7m, and we consider a single road
        self.x = [x_0] # trafic volume
        self.uref = uref

    # demand function
    def d_fcn(self):
        return (min(self.v/self.l*self.x[-1],self.fmax))
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

        # Create nodes as items of class Sommet
        sommets_objets = {name: Sommet(name) for name in self.vertices}

        # Add nodes to graph
        for sommet in sommets_objets.values():
            G.add_node(sommet)

        # Add edges
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
        self.verif_f=[0]
        self.value_s_fcn=[0]
        self.k_old = 0
        self.t_old = 0
        self.T_index = {}
        self.X_index = {}

    # cell i: headnode
    # cell j: tailnode
    # k: variable to plot the flow f23
    # Computes the flow fij
    def f(self,i,j,k):
        if i.startswith("T"):
            return (min(float("inf"),self.X[j].s_fcn()))
        else:
            if i == "X2" and j == "X3" and k != self.k_old:
                self.k_old += 1
                self.value_s_fcn.append(self.X[j].s_fcn())
                # print(self.k_old)
                if self.X[i].d_fcn() < self.X[j].s_fcn():
                    self.verif_f.append(-1)
                    # print("min = d_fcn")
                else:
                    self.verif_f.append(1)
                    # print("min = s_fcn")
            
            return (min(self.X[i].d_fcn(),self.X[j].s_fcn()))
    
    # Computes the flow fij to obtain same result than mpc algo
    def f_mpc(self,i,j,k,Gamma):
        if i.startswith("T"):
            return (min(float("inf"),self.X[j].s_fcn()))
        else:
            if i == "X2" and j == "X3" and k != self.k_old:
                self.k_old += 1
                self.value_s_fcn.append(self.X[j].s_fcn())
                # print(self.k_old)
                if self.X[i].d_fcn() < self.X[j].s_fcn():
                    self.verif_f.append(-1)
                    # print("min = d_fcn")
                else:
                    self.verif_f.append(1)
                    # print("min = s_fcn")
            
            return (min(Gamma[self.X_index[i], k]*self.X[i].d_fcn(),self.X[j].s_fcn()))

    # Defines the flow of the last cell => equal to demand function
    def f_end(self,i):
        return (self.X[i].d_fcn())
    
    # Defines the flow of the last cell to obtain same result than mpc algo
    def f_end_mpc(self,i,k,Gamma):
        return (Gamma[self.X_index[i], k]*self.X[i].d_fcn())
    
    # Defines the flow of a tank as a fraction of the nb of vehicles in the tank, with 0 < u < 1
    def f_tank(self,i,s,u):
        return min(u*self.T[i].x[-1], self.X[s].s_fcn())
    
    # Defines the flow of a tank with uref, a nb of vehicles directly
    def new_f_tank(self,i,s):
        u =min(self.T[i].uref,self.T[i].x[-1])
        return min(u, self.X[s].s_fcn())

    
    # Computes the outflow of a tank to obtain same result than mpc algo
    def new_f_tank_mpc(self,i,s,k,Alpha):
        u =min(self.T[i].uref,self.T[i].x[-1])
        return min(Alpha[self.T_index[i], k]*u, self.X[s].s_fcn())

    # Decides the value of uref to empty the tanks one after another
    def command_manager(self,tanks_list,uref):
        # tanks_list = list(self.T.keys())
        if len(tanks_list) != 0:
            emptying_tank = tanks_list[-1]
            if self.T[emptying_tank].x[-1] < 1:
                self.T[emptying_tank].uref = 0
                tanks_list.pop()
            else:
                self.T[emptying_tank].uref = uref
                # emptying_tank = tanks_list[-1]
    
    # Gives the values of uref found by mpc algo
    def command_manager_mpc(self, tanks_list, u, k):
        # nb of rows and columns
        n_rows, n_col = u.shape
        for i in range(n_rows):
            self.T[tanks_list[i]].uref = float(u[i,k])

            # print("u[i,k] =", u[i,k], type(u[i,k]))
            # print("uref =", self.T[tanks_list[i]].uref, type(self.T[tanks_list[i]].uref))

        
        
        # if len(self.T[tanks_list(0)].x[-1]) == 1:
        #     i_min = 0
        #     mini = 1000
        #     for i,t in enumerate(tanks_list):
        #         desc = len(nx.descendants(self.graph, t))
        #         if desc<min:
        #             mini = desc
        #             i_min = i
        #         self.T[tanks_list(i)].uref = 5
        # for i,t in enumerate(tanks_list):
        #     # choose the furthest tank to empty first
        #     if self.T[t].x[-1] == 0:
        #         self.T[t].uref = 0
        #         tanks_list.pop(i)
        #     else:
                
    

    # Paramètres simulation: h = disc. step, N = nb of steps
    def setting(self, v_properties):
        self.T = {}
        self.X = {}
        for i in self.graph.nodes():
            if i.name.startswith("T"):
                print(f"{i.name} is a tank")
                # print(v_properties[i.nom])
                self.T[i.name] = Cell(i.name,0,0,0,0,*v_properties[i.name])
                print(v_properties[i.name][0])
            else:
                print(f"{i.name} is a portion of road")
                self.X[i.name] = Cell(i.name,*v_properties[i.name])
        # print(self.T)
        # print(self.X)
        self.X_index = {name: idx for idx, name in enumerate(self.X.keys())}
        self.T_index = {name: idx for idx, name in enumerate(self.T.keys())}
        return self.T, self.X, self.X_index, self.T_index
        
    def simu(self, h, N):
        # i = self.X["X1"]
        # l = list(self.graph.predecessors(self.sommets[i.name]))
        # print("Prédécesseurs de " + str(i.name) + ": ", l)
        # d = list(self.graph.successors(self.sommets[i.name]))
        # print("Successeurs de " + str(i.name) + ": ", d)
        # for z in d:
        #     # print(type(z.nom))
        #     print(self.X[z.name])
        tanks_list = list(self.T.keys())
        for k in range (1,N):
            self.command_manager(tanks_list,0.3) # Commande max fonctionnement sans bouchons: uref = 0.3
            for t in self.T:
                # calculer u
                d = list(self.graph.successors(self.sommets[t]))
                self.T[t].x.append(self.T[t].x[-1] - sum(h*(self.new_f_tank(t,s.name)) for s in d)) 
            for i in self.X:
                l = list(self.graph.predecessors(self.sommets[i]))
                # print("Voisins de " + str(i.name) + ": ", l)
                d = list(self.graph.successors(self.sommets[i]))
                if len(d) != 0:
                    somme = 0
                    for j in l:
                        if j.name.startswith("T"):
                            somme += self.new_f_tank(j.name,i)
                        else:
                            somme += self.f(j.name,i,k)

                    self.X[i].x.append(self.X[i].x[-1] + h*(somme - sum(self.f(i,j.name,k) for j in d))) # ordre important
                else:
                    self.X[i].x.append(self.X[i].x[-1] + h*(sum(self.f(j.name,i,k) for j in l) - self.f_end(i))) # ordre important
    
    def simu_mpc(self, h, N, u, Gamma, Alpha):
        tanks_list = list(self.T.keys())
        for k in range (N):
            self.command_manager_mpc(tanks_list, u, k-1)
            for t in self.T:
                d = list(self.graph.successors(self.sommets[t]))
                self.T[t].x.append(self.T[t].x[-1] - sum(h*(self.new_f_tank_mpc(t,s.name,k,Alpha)) for s in d)) 
            for i in self.X:
                l = list(self.graph.predecessors(self.sommets[i]))
                # print("Voisins de " + str(i.name) + ": ", l)
                d = list(self.graph.successors(self.sommets[i]))
                if len(d) != 0:
                    somme = 0
                    for j in l:
                        if j.name.startswith("T"):
                            somme += self.new_f_tank_mpc(j.name,i,k,Alpha)
                        else:
                            somme += self.f_mpc(j.name,i,k,Gamma)

                    self.X[i].x.append(self.X[i].x[-1] + h*(somme - sum(self.f_mpc(i,j.name,k,Gamma) for j in d))) # ordre important
                else:
                    self.X[i].x.append(self.X[i].x[-1] + h*(sum(self.f_mpc(j.name,i,k,Gamma) for j in l) - self.f_end_mpc(i,k,Gamma))) # ordre important


    def verif_simu(self, h, M, N, nh, nt, nc, V, W, L, Cap, Fmax, Mt, Mc):
        # Variables to store the values of flow, respectively for tanks and cells
        Ft = cp.Variable((nt, nh))
        Fc = cp.Variable((nc, nh))
        # Variables to store values of supply function and demand function
        Sf = cp.Variable((nc, nh))
        Df = cp.Variable((nc, nh))
        # Initial nb of vehicles in the tanks and in the cells
        Xt0 = np.array([100, 100], dtype=float).reshape(-1,1)
        Xc0 = np.array([0, 0, 0, 0, 0, 0], dtype=float).reshape(-1,1)
        # Constants converted in 1D
        Xt0_1d = cp.Constant(Xt0)   # shape (2,)
        Xc0_1d = cp.Constant(Xc0)  # shape (6,)

        X0 = cp.vstack([Xt0_1d, Xc0_1d])                 # shape (8,)
        print(X0.shape)

        # Describe the convex problem
        X = cp.Variable((nt+nc, nh+1))
        
        Gamma = np.zeros((nc, M))
        Alpha = np.zeros((nt, M))

        # Complete vector U of commands
        U = cp.Variable((nt, nh))

        ######### Cost function #########
        # Now we can try to penalize the number of vehicles present in the whole network
        obj = cp.Minimize(cp.sum(X))
        ######### We store states and inputs in tables #########
        X_hist = np.zeros((nt+nc, N+1))  
        X_hist[:, 0] = X0.value.flatten(order='C')
        print(X_hist[:, 0].shape)
        U_hist = np.zeros((nt, N))    


        for k_simu in range(0, N, M):
            print(k_simu)

            ####### Constraints #########
            constr = []
            # Dynamics
            constr += [X[:, 0:1] == X0, X[:, 1:] == X[:,:nh] + h*(Mt@Ft + Mc@Fc)]
            for k in range(nh):
                constr += [Sf[:, k] <= (Cap - W @ X[2:8, k])]
                # constr += [Sf[:, k] >= 0]
                constr += [U[:, k] >= 0, U[:, k] <= X[0:2, k]]
                constr += [Ft[:, k] <= cp.vstack([Sf[0, k], Sf[3, k]])]
                for i in range(nc):
                    # constr += [Z[i, k] <= x_max * Gamma[i, k],
                    #         Z[i, k] <= X[2+i, k]] # Gamma * X
                    # constr += [Df[i,k] <= Z[i, k] * V[i]/L[i]]
                    constr += [Df[i,k] <= V[i]/L[i]*X[2+i, k]]
                constr += [Fc[5, k] == Df[5, k]]
                constr += [Fc[0:5, k] <= Sf[1:6, k]]
                constr += [Fc[0:5, k] <= Df[0:5, k]]

            constr += [Df <= Fmax]
            constr += [Ft >= 0]
            constr += [Fc >= 0]
            constr += [Ft <= U]
            # constr += [Gamma >= 0, Gamma <= 1]
            prob = cp.Problem(obj, constr)

            prob.solve(warm_start=True)

            # print("U.value =", U.value)

            u = U.value[:,0:M]
            
            U_hist[:, k_simu:k_simu+M] = u
            X_hist[:, k_simu+1:k_simu+M+1] = X.value[:,0:M]

            # We set X0 to the last vector X given by the MPC algo
            X0 = X.value[:,M-1].reshape(-1,1) 

            # We compute Gamma
            for i in range(nc):
                for j in range(M):
                    Gamma[i,j] = float(Fc[i,j].value/Df[i,j].value)
            # We compute Alpha
            for i in range(nt):
                for j in range(M):
                    Alpha[i,j] = float(Ft[i,j].value/U[i,j].value)
            # We simulate the system
            self.simu_mpc(h, M, u, Gamma, Alpha)

            # We set X0 to the last states X computed by the simulator
            # for i in self.T:
            #     Xt0[self.T_index[i],0] = self.T[i].x[-1]
            # for i in self.X:
            #     Xc0[self.X_index[i],0] = self.X[i].x[-1]
            # Xt0_1d = cp.Constant(Xt0)
            # Xc0_1d = cp.Constant(Xc0)
            # X0 = cp.vstack([Xt0_1d, Xc0_1d])


        ########## Update sequences of states and inputs ############
        X_hist = X_hist
        U_hist = U_hist
        print(X_hist.shape)
        return X_hist, U_hist


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

    # Displays with subplots
    # dict: dictionnary containing the data (type Cell)
    # t: time vector
    def create_subplot(self, dict, t):
        # Nombre de variables
        n = len(dict)

        # Déterminer la grille (ici 2 colonnes)
        cols = 2
        rows = (n + cols - 1) // cols  # arrondi vers le haut

        fig, axs = plt.subplots(rows, cols, figsize=(10, 6))

        # axs est une matrice → on la transforme en liste pour itérer facilement
        axs = axs.flatten()

        # Boucle sur chaque variable
        for i, var in enumerate(dict):
            axs[i].plot(t,dict[var].x)
            axs[i].set_title(var)

        # Supprimer les subplots vides si n n'est pas multiple de cols
        for j in range(i+1, len(axs)):
            fig.delaxes(axs[j])

        plt.tight_layout()
        plt.show()

    def create_subplot_mpc(self, table, t, label):
        n_rows, n_col = table.shape

        # Déterminer la grille (ici 2 colonnes)
        cols = 2
        rows = (n_rows + cols - 1) // cols  # arrondi vers le haut

        fig, axs = plt.subplots(rows, cols, figsize=(10, 6))

        # axs est une matrice → on la transforme en liste pour itérer facilement
        axs = axs.flatten()

        # Boucle sur chaque variable
        for i in range(n_rows):
            axs[i].plot(t,table[i,:])
            axs[i].set_title(label + str(i+1) + " MPC")
        
        # Supprimer les subplots vides si n n'est pas multiple de cols
        for j in range(i+1, len(axs)):
            fig.delaxes(axs[j])

        plt.tight_layout()
        plt.show()


    # To display with subplots
    def results_2(self, h, N):
        t_inputs = np.arange(N+1)*h
        t_states = np.arange(N+1)*h
        
        plt.figure
        # plt.plot(t,self.value_s_fcn)
        # plt.plot(t,self.verif_f)
        # plt.title("y(t) = -1 if f_23(x)=d_fcn // y(t) = +1 if f_23(x)=s_fcn")
        # plt.show()

        self.create_subplot(self.T, t_inputs)
        self.create_subplot(self.X, t_states)


    # To display with subplots
    def results_2_mpc(self, h, N, tab_u, tab_x):
        t_inputs = np.arange(N)*h
        t_states = np.arange(N+1)*h
        
        plt.figure
        # plt.plot(t,self.value_s_fcn)
        # plt.plot(t,self.verif_f)
        # plt.title("y(t) = -1 if f_23(x)=d_fcn // y(t) = +1 if f_23(x)=s_fcn")
        # plt.show()

        self.create_subplot(self.T, t_states)
        self.create_subplot(self.X, t_states)

        self.create_subplot_mpc(tab_x[0:2,:], t_states, "T")
        self.create_subplot_mpc(tab_x[2:,:], t_states, "X")
        # self.create_subplot_mpc(tab_u, t_inputs)
        


                


