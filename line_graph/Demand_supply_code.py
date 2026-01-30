import numpy as np
import matplotlib.pyplot as plt

# Paramètres (exemple)
v = 70 / 3.6          # vitesse
l = 200               # longueur de la cellule
C = 1000 / 3600       # capacité maximale
gamma = 1             # paramètre de contrôle

w = 20 / 3.6          # coefficient de saturation
c = w * l / 4.7       # constante de capacité

# Domaine de x
x = np.linspace(0, 60, 300)

# Fonctions demande et supply
demand = gamma * np.minimum((v / l) * x, C)
supply = np.maximum(c - w * x, 0)

# Tracé de la demande
plt.figure()
plt.plot(x, demand, linewidth=2)
plt.grid(True)
plt.xlabel("x")
plt.ylabel("Demand")
plt.title("Demand function")
plt.show()

# Tracé de la supply
plt.figure()
plt.plot(x, supply, linewidth=2)
plt.grid(True)
plt.xlabel("x")
plt.ylabel("Supply")
plt.title("Supply function")
plt.show()