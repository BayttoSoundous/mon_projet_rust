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
> ⚠️ **GND COMMUN obligatoire** : relier ensemble les masses Arduino + L298N + encodeur + batterie.

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
