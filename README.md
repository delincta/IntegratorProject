# Modélisation mathématique du réseau hydraulique discret

On considère un réseau orienté $G=(V,E)$ composé de deux types de sommets :

* $\mathcal{T}$ : ensemble des **cuves** (réservoirs),
* $\mathcal{X}$ : ensemble des **cellules de transport** (tunnel).

On a $V = \mathcal{T} \cup \mathcal{X}$ et $\mathcal{T} \cap \mathcal{X} = \emptyset$.

Le temps est discrétisé avec un pas $h > 0$ :

$$
t_k = k h, \quad k = 0,1,\dots,K.
$$

---

## 1. Variables d'état et de commande

Pour tout $k$ :

* $x_T^k$ : volume d’eau dans la cuve $T \in \mathcal{T}$,
* $x_i^k$ : volume d’eau dans la cellule $i \in \mathcal{X}$,
* $u_T^k \in [0,1]$ : commande d’ouverture de la vanne de la cuve $T$,
* $f_{ij}^k \ge 0$ : flux entre les sommets $i \to j$,
* $\mu_i^k$ : débit de sortie hors réseau pour les cellules terminales.

---

## 2. Fonctions de demande et d’offre

### Fonction de demande (cellules)

$$
d_i(x_i^k) = \min\left( \frac{v_i}{l_i} x_i^k ,; C_i \right),
$$

avec $v_i$ la vitesse, $l_i$ la longueur et $C_i$ la capacité maximale.

### Fonction d’offre (supply)

$$
s_i(x_i^k) = \max(c_i - w_i x_i^k ,; 0),
$$

avec $c_i$ la capacité maximale et $w_i$ la pente de saturation.

---

## 3. Définition des flux

### Flux cellule → cellule

$$
f_{ij}^k = \min\left(d_i(x_i^k),; s_j(x_j^k)\right), \quad i,j \in \mathcal{X}.
$$

### Flux cuve → cellule

$$
f_{Tj}^k = \min\left(u_T^k , x_T^k,; s_j(x_j^k)\right),
\quad T \in \mathcal{T},\ j \in \mathcal{X}.
$$

### Sortie hors réseau

Si une cellule $i$ n’a pas de successeur :

$$
\mu_i^k = d_i(x_i^k).
$$

### Interdiction de retour vers les cuves

$$
f_{jT}^k = 0, \quad \forall j \in \mathcal{X},\ \forall T \in \mathcal{T}.
$$

---

## 4. Dynamiques discrètes du système

### Évolution des cuves

$$
x_T^{k+1} = x_T^k + h \sum_{j: T \to j} f_{Tj}^k
$$

### Évolution des cellules

$$
x_i^{k+1} = x_i^k + h  \left(
  \sum_{j \in \{ j \mid j \to i \}} f_{ji}^k-\sum_{m \in \{ m \mid i \to m \}} f_{im}^k-\mu_i^k \right)
, \quad i \in \mathcal{X}.
$$



---

## 5. Contraintes physiques

$$
\begin{cases}
x_i^k \ge 0, \quad x_T^k \ge 0, \
f_{ij}^k \ge 0, \
0 \le u_T^k \le 1.
\end{cases}
$$

---

## 6. Problème de contrôle optimal sur horizon fini

On choisit un coût correspondant au temps total passé dans le réseau (comme dans l’article) :

$$
J = \sum_{k=0}^{K} \left( \sum_{i \in \mathcal{X}} x_i^k + \sum_{T \in \mathcal{T}} x_T^k \right)
$$

Sous contraintes :

- dynamiques du système,
- contraintes de flux :

$$
\sum_{m:\,i \to m} f_{im}^k \le d_i(x_i^k),
$$

$$
\sum_{j:\,j \to i} f_{ji}^k \le s_i(x_i^k),
$$

$$
\sum_{j:\,T \to j} f_{Tj}^k \le x_T^k,
$$

- pas de retour vers les cuves :

$$
f_{jT}^k = 0,
$$

- condition initiale :

$$
x^0 \text{ donné.}
$$

---

## 7. Relaxation convexe

Afin de rendre le problème convexe, les flux $f_{ij}^k$ sont traités comme variables indépendantes et les fonctions $\min(\cdot)$ sont remplacées par des contraintes d'inégalité :

$$
0 \le f_{ij}^k \le d_i(x_i^k),
$$

$$
\sum_{j:,j\to i} f_{ji}^k \le s_i(x_i^k).
$$

Les dynamiques deviennent alors affines en $x$ et $f$, ce qui permet de résoudre le problème par programmation quadratique convexe (QP), directement exploitable en MPC.
