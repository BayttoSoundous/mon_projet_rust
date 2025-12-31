# Asservissement de position d’un moteur DC par PID (Arduino + L298N + Encodeur KY-040)



## 4) Équations du projet (détaillées et expliquées)

Cette section présente le **modèle (plant)** du moteur DC, le **calcul de l’erreur**, la **loi PID** et l’analyse **boucle ouverte / boucle fermée** (SFG, Root Locus, forme d’état).

---

### 4.1 Notations (symboles + unités)

| Symbole | Unité | Signification (physique) |
|---|---:|---|
| `V(t)` | V | Tension appliquée aux bornes du moteur |
| `i(t)` | A | Courant dans l’induit |
| `R` | Ω | Résistance de l’induit |
| `L` | H | Inductance de l’induit |
| `J` | kg·m² | Inertie équivalente (moteur + charge) |
| `f` | N·m·s/rad | Frottement visqueux équivalent |
| `K_e` | V·s/rad | Constante FEM (contre-électromotrice) |
| `K_t` | N·m/A | Constante de couple |
| `ω(t)` | rad/s | Vitesse angulaire |
| `θ(t)` | rad ou ° | Position angulaire |
| `θ_ref(t)` | rad ou ° | Consigne (position désirée) |
| `e(t)` | rad ou ° | Erreur de position |
| `u(t)` | — | Signal de commande (sortie PID) |
| `PWM` | — | Rapport cyclique (0–255 sur Arduino) |
| `PPR` | imp/tour | Impulsions par tour de l’encodeur |
| `N(t)` | impulsions | Compteur d’impulsions mesurées |



---

## 4.2 Modèle du moteur DC (Plant) : équations physiques

### 4.2.1 Équation électrique

**Équation :**

V(t) = R*i(t) + L*di(t)/dt + K_e*ω(t)


**À quoi sert cette équation ?**
- Elle relie la tension appliquée `V` au courant `i` et à la vitesse `ω`.
- Le terme `K_e*ω` représente la **FEM de retour** : plus le moteur tourne vite, plus il “s’oppose” à la tension appliquée.

**Simplification souvent utilisée en automatique (si on néglige L) :**


L ≈ 0  =>  i(t) = ( V(t) - K_e*ω(t) ) / R


**Pourquoi cette simplification ?**
- Elle simplifie le modèle (ordre plus faible) et reste souvent suffisante pour une étude position/PID.

---

### 4.2.2 Équation mécanique

**Équation :**

J*dω(t)/dt + f*ω(t) = K_t*i(t)


**À quoi sert cette équation ?**
- Elle décrit comment le couple moteur `K_t*i` accélère le rotor.
- `J*dω/dt` : effet d’inertie (accélération).
- `f*ω` : pertes par frottement visqueux.

---

### 4.2.3 Relation vitesse ↔ position

**Équation :**


ω(t) = dθ(t)/dt


**À quoi sert cette équation ?**
- Elle relie **vitesse** et **position**.
- Elle explique pourquoi un asservissement de position contient naturellement un **intégrateur**.

---

## 4.3 Fonction de transfert du plant (position)

Après combinaison (forme simplifiée usuelle), on obtient un modèle de position :


G(s) = Θ(s) / V(s) = K / ( s * (J*s + f) )


**À quoi sert `G(s)` ?**
- C’est le **plant** utilisé pour :
  - simulation MATLAB/Simulink
  - analyse Root Locus
  - étude de stabilité en boucle fermée
- La présence de `1/s` indique un système de **type position** (intégration).

> Remarque pratique : le driver (L298N) introduit saturation + pertes.  
> On peut l’approximer par un gain : `V(s) ≈ K_drv * PWM(s)`.

---

## 4.4 Mesure de position et calcul de l’erreur (encodeur)

### 4.4.1 Conversion impulsions → angle

En degrés :


θ[deg] = 360 * N / PPR



**À quoi sert cette équation ?**
- Transformer la mesure brute `N` (impulsions) en un angle en **degrés**.

---

### 4.4.2 Consigne en impulsions (utile pour Arduino)



N_ref = ( θ_ref / 360 ) * PPR


**À quoi sert cette équation ?**
- Travailler directement en **impulsions** dans le code (souvent plus simple et robuste).

---

### 4.4.3 Erreur (forme “position” et forme “impulsions”)

Erreur en angle :


e(t) = θ_ref(t) - θ(t)



Erreur en impulsions :


e_N = N_ref - N



**À quoi sert l’erreur ?**
- C’est le signal principal que le PID doit ramener vers 0.

---

### 4.4.4 Choix du sens de rotation (commande logique)

- Si `e_N > 0` : tourner sens "+"
- Si `e_N < 0` : tourner sens "−"
- Si `|e_N| ≤ ε` : arrêt (tolérance)

**Pourquoi une tolérance `ε` ?**
- Parce que le KY-040 est discret et bruité → éviter les oscillations autour de la consigne.

---

## 4.5 Correcteur PID

### 4.5.1 Loi PID (continue)


u(t) = Kp*e(t) + Ki * ∫ e(t) dt + Kd * de(t)/dt



**À quoi sert chaque terme ?**
- `Kp*e` : correction immédiate (réactivité).
- `Ki*∫e dt` : réduit / annule l’erreur finale (erreur statique).
- `Kd*de/dt` : anticipe les variations, réduit dépassement/oscillations.

---

### 4.5.2 Forme dans Laplace



C(s) = Kp + Ki/s + Kd*s


**À quoi sert cette forme ?**
- Calculer la boucle ouverte `L(s)`, la boucle fermée et faire Root Locus.

---

## 4.6 Boucle ouverte et boucle fermée

### 4.6.1 Boucle ouverte (Open-loop)

Sans retour de position :


Θ(s) = G(s) * V(s)


**À quoi sert l’étude boucle ouverte ?**
- Comprendre la dynamique brute du moteur (sans correction).
- Base de l’analyse Root Locus.

Pour Root Locus avec P seul :


L(s) = Kp * G(s) = Kp * K / ( s*(J*s + f) )


---

### 4.6.2 Boucle fermée (Closed-loop)

Avec retour unitaire `H(s)=1` :


Θ(s) / Θ_ref(s) = ( C(s)*G(s) ) / ( 1 + C(s)*G(s) )



**À quoi sert cette équation ?**
- Donne la dynamique réelle de l’asservissement.
- Permet de prédire stabilité, dépassement, temps de réponse.

---

## 4.7 SFG (Signal Flow Graph) + écriture compacte

### 4.7.1 Équations SFG


e = θ_ref - θ
u = C(s) * e
θ = G(s) * u



### X.7.2 Matrice (écriture compacte, équivalente schéma-blocs)
Une écriture possible (forme “structurelle”) :

''' 
[ e ]   [  1   0  -1 ] [ θ_ref ]
[ u ] = [ C   0   0 ] [   e   ]
[ θ ]   [  0   G   0 ] [   u   ]
'''

> Ici `C = C(s)` et `G = G(s)` (notation compacte).

**À quoi sert le SFG ?**
- Représentation mathématique équivalente au schéma-blocs.
- Permet de retrouver la FT globale (ex. formule de Mason).

---

## 4.8 Root Locus (lieu des racines)

Boucle ouverte (P seul) :


L(s) = Kp * K / ( s*(J*s + f) )



**Pôles OL :**
- `s = 0`
- `s = -f/J`

**Zéros OL :**
- aucun

**À quoi sert le Root Locus ?**
- Voir comment les pôles BF se déplacent quand `Kp` varie.
- Choisir `Kp` pour une réponse stable (pôles dans le demi-plan gauche).

---

## 4.9 Forme d’état : matrices du système (plant)

### 4.9.1 États


x1 = θ
x2 = ω



### 4.9.2 Équations d’état


x1_dot = x2
x2_dot = -(f/J)*x2 + (K/J)*u



### 4.9.3 Matrices


x_dot = A*x + B*u
y     = C*x + D*u



A = [ 0     1   ]
[ 0   -f/J  ]

B = [ 0   ]
[ K/J ]

C = [ 1   0 ]

D = [ 0 ]



**À quoi sert la forme d’état ?**
- Analyse stabilité via valeurs propres.
- Base pour contrôle avancé (retour d’état).

---
## 5) Partie Arduino : Code avant PID / après PID

Cette section présente les deux versions du programme Arduino utilisées dans ce projet :
- une version **avant PID** (commande simple du moteur)
- une version **après PID** (asservissement avec correcteur PID et retour encodeur)

### 1) Code Arduino AVANT PID (Boucle ouverte)

Dans cette version, le moteur est commandé directement via le driver **L298N** (PWM pour la vitesse + IN3/IN4 pour le sens).  
La commande est appliquée **sans retour de mesure**, donc il n’y a **aucune correction d’erreur**.

 Résultat : le moteur peut tourner mais **ne garantit pas** l’arrêt exactement à l’angle demandé (ex : 180°), car la vitesse dépend de la charge, de la tension batterie et des frottements.

*(Colle ici ton code Arduino AVANT PID)*

### 2) Code Arduino APRÈS PID (Boucle fermée)

Dans cette version, on ajoute un **retour encodeur** pour mesurer la position réelle du moteur (angle).  
L’angle mesuré est comparé à la consigne, puis l’erreur est corrigée par un **PID** qui ajuste automatiquement la commande envoyée au moteur.

 Résultat : le moteur atteint la consigne (ex : 180°) avec une meilleure précision et **s’arrête** lorsque l’erreur devient faible (tolérance).

*(Colle ici ton code Arduino APRÈS PID)*

### Rôle des gains PID (rappel rapide)

- **Kp** : augmente la rapidité de réponse (réduit l’erreur rapidement), mais peut créer du dépassement si trop grand.  
- **Ki** : supprime l’erreur statique (offset), mais peut provoquer une oscillation si trop élevé.  
- **Kd** : réduit les oscillations et le dépassement (effet “frein”), améliore la stabilité.
## X.7 SFG (Signal Flow Graph) + écriture compacte

### X.7.1 Équations SFG (relations du schéma-blocs)
On peut écrire la boucle de régulation sous forme de 3 relations :

\[
e(s)=\Theta_{\text{ref}}(s)-\Theta(s)
\]
\[
u(s)=C(s)\,e(s)
\]
\[
\Theta(s)=G(s)\,u(s)
\]

---

### X.7.2 Écriture matricielle compacte (forme structurelle)
En empilant les variables \(e, u, \Theta\), une écriture compacte possible est :

\[
\begin{bmatrix}
e \\
u \\
\Theta
\end{bmatrix}
=
\begin{bmatrix}
1 & 0 & -1 \\
C(s) & 0 & 0 \\
0 & G(s) & 0
\end{bmatrix}
\begin{bmatrix}
\Theta_{\text{ref}} \\
e \\
u
\end{bmatrix}
\]

avec \(C=C(s)\) et \(G=G(s)\).

> Remarque : cette écriture est une **notation structurelle** (compacte) qui reflète le chaînage :
> \(\Theta_{\text{ref}} \rightarrow e \rightarrow u \rightarrow \Theta\) et le retour \(\Theta \rightarrow e\).

---

### À quoi sert le SFG ?
- Donner une représentation mathématique équivalente au schéma-blocs.
- Retrouver la fonction de transfert globale (ex. via la formule de Mason).
- Clarifier les dépendances entre signaux \( \Theta_{\text{ref}}, e, u, \Theta \).

