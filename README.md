# Asservissement de position d’un moteur DC par PID (Arduino + L298N + Encodeur KY-040)

**Modélisation • Boucle ouverte/fermée • SFG • Root Locus • Simulation • Implémentation**

---

## 0) Résumé du projet
Ce dépôt présente un **asservissement de position angulaire** pour un **moteur DC**.  
La **position** est mesurée grâce à un **encodeur incrémental KY-040**, puis comparée à une **consigne** (ex. 30°, 90°, 180°).  
L’erreur est corrigée par un **PID**, qui génère une commande appliquée au moteur via un pont en H **L298N** :

- **vitesse** par **PWM**
- **sens** par inversion (IN3/IN4)

---

## 1) Cahier des charges

### 1.1 Objectif
- Atteindre une **position angulaire demandée** (ex. 30°, 90°, 180°) et **s’y maintenir**.

### 1.2 Fonctions attendues
- Définir une **consigne** de position : `theta_ref`
- Mesurer la position réelle : `theta`
- Calculer l’erreur :
  - `e = theta_ref - theta`
- Calculer la commande PID :
  - `u = Kp*e + Ki*integral(e*dt) + Kd*(de/dt)`
- Commander le moteur via le L298N :
  - **sens** (IN3/IN4)
  - **vitesse** (ENB en PWM)

### 1.3 Contraintes / limites réelles
- **Encodeur KY-040** : faible résolution → **PPR = 29** impulsions par tour (mesuré expérimentalement)
- **L298N** : pertes (chute de tension) + saturation à forte charge
- Système réel : **frottements**, **jeux mécaniques**, **bruit** sur la mesure

---

## 2) Rôle de chaque composant
- **Arduino** : lecture encodeur + calcul PID + génération PWM
- **L298N** : interface de puissance + inversion de sens + entrée PWM
- **Moteur DC** : actionneur
- **Encodeur KY-040** : retour d’information (feedback)
- **Alimentation 7,4 V** : fournit le courant du moteur (**ne pas alimenter le moteur via le 5V Arduino**)

---

## 3) Branchement (propre et reproductible)
>  **GND COMMUN obligatoire** : relier ensemble les masses Arduino + L298N + encodeur + batterie.

### 3.1 Moteur ↔ L298N
- Fil moteur 1 → **OUT3**
- Fil moteur 2 → **OUT4**

### 3.2 L298N ↔ Arduino
- **IN3 → D8**
- **IN4 → D9**
- **ENB → D10 (PWM)**
- **GND → GND Arduino**

### 3.3 Batterie ↔ L298N
- Batterie **+7,4 V** → **+12V** (L298N)
- Batterie **−** → **GND** (L298N)

### 3.4 Encodeur KY-040 ↔ Arduino
- **CLK → D2** (interruption)
- **DT  → D3**
- **+   → 5V**
- **GND → GND**
-
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

> Dans notre montage : **`PPR = 29`** (mesuré expérimentalement).

---

## 4.2 Modèle du moteur DC (Plant) : équations physiques

### 4.2.1 Équation électrique

**Équation :**
```

V(t) = R*i(t) + L*di(t)/dt + K_e*ω(t)

```

**À quoi sert cette équation ?**
- Elle relie la tension appliquée `V` au courant `i` et à la vitesse `ω`.
- Le terme `K_e*ω` représente la **FEM de retour** : plus le moteur tourne vite, plus il “s’oppose” à la tension appliquée.

**Simplification souvent utilisée en automatique (si on néglige L) :**
```

L ≈ 0  =>  i(t) = ( V(t) - K_e*ω(t) ) / R

```

**Pourquoi cette simplification ?**
- Elle simplifie le modèle (ordre plus faible) et reste souvent suffisante pour une étude position/PID.

---

### 4.2.2 Équation mécanique

**Équation :**
```

J*dω(t)/dt + f*ω(t) = K_t*i(t)

```

**À quoi sert cette équation ?**
- Elle décrit comment le couple moteur `K_t*i` accélère le rotor.
- `J*dω/dt` : effet d’inertie (accélération).
- `f*ω` : pertes par frottement visqueux.

---

### 4.2.3 Relation vitesse ↔ position

**Équation :**
```

ω(t) = dθ(t)/dt

```

**À quoi sert cette équation ?**
- Elle relie **vitesse** et **position**.
- Elle explique pourquoi un asservissement de position contient naturellement un **intégrateur**.

---

## 4.3 Fonction de transfert du plant (position)

Après combinaison (forme simplifiée usuelle), on obtient un modèle de position :

```

G(s) = Θ(s) / V(s) = K / ( s * (J*s + f) )

```

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
```

θ[deg] = 360 * N / PPR

```

**À quoi sert cette équation ?**
- Transformer la mesure brute `N` (impulsions) en un angle en **degrés**.

---

### 4.4.2 Consigne en impulsions (utile pour Arduino)

```

N_ref = ( θ_ref / 360 ) * PPR

```

**À quoi sert cette équation ?**
- Travailler directement en **impulsions** dans le code (souvent plus simple et robuste).

---

### 4.4.3 Erreur (forme “position” et forme “impulsions”)

Erreur en angle :
```

e(t) = θ_ref(t) - θ(t)

```

Erreur en impulsions :
```

e_N = N_ref - N

```

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

```

u(t) = Kp*e(t) + Ki * ∫ e(t) dt + Kd * de(t)/dt

```

**À quoi sert chaque terme ?**
- `Kp*e` : correction immédiate (réactivité).
- `Ki*∫e dt` : réduit / annule l’erreur finale (erreur statique).
- `Kd*de/dt` : anticipe les variations, réduit dépassement/oscillations.

---

### 4.5.2 Forme dans Laplace

```

C(s) = Kp + Ki/s + Kd*s

```

**À quoi sert cette forme ?**
- Calculer la boucle ouverte `L(s)`, la boucle fermée et faire Root Locus.

---

## 4.6 Boucle ouverte et boucle fermée

### 4.6.1 Boucle ouverte (Open-loop)

Sans retour de position :
```

Θ(s) = G(s) * V(s)

```

**À quoi sert l’étude boucle ouverte ?**
- Comprendre la dynamique brute du moteur (sans correction).
- Base de l’analyse Root Locus.

Pour Root Locus avec P seul :
```

L(s) = Kp * G(s) = Kp * K / ( s*(J*s + f) )

```

---

### 4.6.2 Boucle fermée (Closed-loop)

Avec retour unitaire `H(s)=1` :
```

Θ(s) / Θ_ref(s) = ( C(s)*G(s) ) / ( 1 + C(s)*G(s) )

```

**À quoi sert cette équation ?**
- Donne la dynamique réelle de l’asservissement.
- Permet de prédire stabilité, dépassement, temps de réponse.

---

## 4.7 SFG (Signal Flow Graph) + écriture compacte

### 4.7.1 Équations SFG
```

e = θ_ref - θ
u = C(s) * e
θ = G(s) * u

```

### 4.7.2 Matrice (écriture compacte, équivalente schéma-blocs)
Une écriture possible (forme “structurelle”) :
```

[ e ]   [  1   0  -1 ] [ θ_ref ]
[ u ] = [ C   0   0 ] [   e   ]
[ θ ]   [  0   G   0 ] [   u   ]

```
> Ici `C = C(s)` et `G = G(s)` (notation compacte).

**À quoi sert le SFG ?**
- Représentation mathématique équivalente au schéma-blocs.
- Permet de retrouver la FT globale (ex. formule de Mason).

---

## 4.8 Root Locus (lieu des racines)

Boucle ouverte (P seul) :
```

L(s) = Kp * K / ( s*(J*s + f) )

```

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
```

x1 = θ
x2 = ω

```

### 4.9.2 Équations d’état
```

x1_dot = x2
x2_dot = -(f/J)*x2 + (K/J)*u

```

### 4.9.3 Matrices
```

x_dot = A*x + B*u
y     = C*x + D*u

```
```

A = [ 0     1   ]
[ 0   -f/J  ]

B = [ 0   ]
[ K/J ]

C = [ 1   0 ]

D = [ 0 ]

```

**À quoi sert la forme d’état ?**
- Analyse stabilité via valeurs propres.
- Base pour contrôle avancé (retour d’état).

---

## 4.10 Matrices Root Locus (boucle fermée avec P seul)

Commande :
```

u = Kp * ( r - y )

```

Boucle fermée :
```

x_dot = (A - B*Kp*C) * x + B*Kp*r

```

Matrice dynamique :
```

A_cl = [ 0           1   ]
[ -(K*Kp)/J  -f/J ]

```

Équation caractéristique :
```

det( sI - A_cl ) = 0
=> s^2 + (f/J)*s + (K*Kp)/J = 0

```

**À quoi sert cette équation ?**
- Les racines sont les pôles BF.
- Root Locus = évolution de ces racines quand `Kp` varie.

---

## 4.11 Extension PID (état intégrateur) — modèle augmenté

État intégrateur :
```

x3 = ∫ (r - y) dt
=> x3_dot = r - x1

```

Système augmenté (structure générale) :
```

x_dot = A_aug * x + B_aug * u + E_aug * r

```

Avec (exemple cohérent avec x = [x1 x2 x3]^T) :
```

A_aug = [  0     1     0 ]
[  0   -f/J    0 ]
[ -1     0     0 ]

B_aug = [  0   ]
[ K/J  ]
[  0   ]

E_aug = [ 0 ]
[ 0 ]
[ 1 ]

```

Interprétation “PID” (idée) :
- `Kp` agit sur l’erreur (liée à `x1`)
- `Kd` agit sur la vitesse (liée à `x2`)
- `Ki` agit sur l’intégrale (liée à `x3`)

**À quoi sert cette extension ?**
- Représenter le terme intégral `Ki*∫e dt` comme un état.
- Étudier stabilité/amortissement par valeurs propres.

---

## 4.12 PID discret (Arduino) — lien théorie ↔ code

Avec un pas d’échantillonnage `dt` :

Erreur (en impulsions) :
```

e[k] = N_ref - N[k]

```

Intégrale (somme) :
```

I[k] = I[k-1] + e[k] * dt

```

Dérivée :
```

D[k] = ( e[k] - e[k-1] ) / dt

```

Commande PID :
```

u[k] = Kp*e[k] + Ki*I[k] + Kd*D[k]

```

Saturation PWM (0..255) :
```

PWM[k] = sat( |u[k]| )
sat(x) = min( max(x, 0), 255 )

```

Sens de rotation :
- si `u[k] > 0` : sens "+"
- si `u[k] < 0` : sens "−"
- si `|e[k]| ≤ ε` : stop (tolérance)

**À quoi sert cette partie ?**
- Justifier la correspondance entre équations continues et le code Arduino.
- Expliquer que la saturation + le bruit imposent une tolérance `ε`.
```

