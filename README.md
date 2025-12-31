## 4) Équations & matrices (version propre GitHub)

### 4.2 Modèle du moteur DC (plant)

#### 4.2.1 Équation électrique
\[
V(t)=R\,i(t)+L\frac{di(t)}{dt}+K_e\,\omega(t)
\]

#### 4.2.2 Équation mécanique
\[
J\frac{d\omega(t)}{dt}+f\,\omega(t)=K_t\,i(t)
\]

#### 4.2.3 Relation vitesse ↔ position
\[
\omega(t)=\frac{d\theta(t)}{dt}
\]

---

### 4.3 Fonction de transfert (position)

#### 4.3.1 Modèle complet (avec l’inductance \(L\))
En combinant les équations (entrée = tension \(V\), sortie = position \(\theta\)) :
\[
\frac{\Theta(s)}{V(s)}=\frac{K_t}{s\Big((J s+f)(L s+R)+K_eK_t\Big)}
\]

#### 4.3.2 Modèle simplifié usuel (si \(L \approx 0\))
Si l’inductance est négligeable :
\[
i(t)=\frac{V(t)-K_e\omega(t)}{R}
\]

Substitution dans l’équation mécanique :
\[
J\dot{\omega}(t)+\Big(f+\frac{K_tK_e}{R}\Big)\omega(t)=\frac{K_t}{R}V(t)
\]

Donc en Laplace :
\[
\frac{\Omega(s)}{V(s)}=\frac{K_t/R}{J s+\Big(f+\frac{K_tK_e}{R}\Big)}
\]
\[
G(s)=\frac{\Theta(s)}{V(s)}=\frac{K_t/R}{s\left(J s+\left(f+\frac{K_tK_e}{R}\right)\right)}
\]

#### 4.3.3 L298N (approximation simple)
Le L298N + l’alimentation introduisent saturation et pertes. On peut approximer :
\[
V(t)\approx K_{\mathrm{drv}}\,\mathrm{PWM}(t)
\]
avec \(\mathrm{PWM}\in[0,255]\) (Arduino).

---

### 4.4 Encodeur (KY-040) : impulsions ↔ angle

#### 4.4.1 Impulsions → angle
En degrés :
\[
\theta[^\circ]=360\,\frac{N}{\mathrm{PPR}}
\]

En radians :
\[
\theta[\mathrm{rad}]=2\pi\,\frac{N}{\mathrm{PPR}}
\]

#### 4.4.2 Consigne angle → consigne impulsions
\[
N_{\mathrm{ref}}=\frac{\theta_{\mathrm{ref}}}{360}\,\mathrm{PPR}
\]

#### 4.4.3 Erreur
Erreur en angle :
\[
e(t)=\theta_{\mathrm{ref}}(t)-\theta(t)
\]

Erreur en impulsions (très pratique dans le code) :
\[
e_N=N_{\mathrm{ref}}-N
\]

---

### 4.5 PID (forme continue + forme discrète Arduino)

#### 4.5.1 PID continu
\[
u(t)=K_p\,e(t)+K_i\int e(t)\,dt+K_d\,\frac{de(t)}{dt}
\]
\[
C(s)=K_p+\frac{K_i}{s}+K_d\,s
\]

#### 4.5.2 PID discret (échantillonné) — version “Arduino-friendly”
Soit \(T_s\) la période d’échantillonnage (ex. 0.01 s).

\[
e[k]=r[k]-y[k]
\]
\[
I[k]=I[k-1]+e[k]\,T_s
\]
\[
D[k]=\frac{e[k]-e[k-1]}{T_s}
\]
\[
u[k]=K_p\,e[k]+K_i\,I[k]+K_d\,D[k]
\]

Saturation PWM (commande moteur) :
\[
\mathrm{PWM}[k]=\mathrm{sat}_{[0,255]}\big(|u[k]|\big)
\]

Choix du sens :
- si \(u[k]\ge 0\) → sens “+”
- si \(u[k]<0\) → sens “−”

Arrêt avec tolérance :
\[
|e_N|\le \varepsilon \;\Rightarrow\; \mathrm{PWM}=0
\]

---

### 4.6 Boucle ouverte / boucle fermée

Boucle ouverte :
\[
L(s)=C(s)\,G(s)
\]

Boucle fermée (retour unitaire \(H(s)=1\)) :
\[
\frac{\Theta(s)}{\Theta_{\mathrm{ref}}(s)}=\frac{C(s)\,G(s)}{1+C(s)\,G(s)}
\]

---

### 4.9 Forme d’état (matrices du plant)

#### 4.9.1 Modèle 2 états (simplifié, \(L\approx 0\))
États :
\[
x_1=\theta,\quad x_2=\omega,\quad u=V,\quad y=\theta
\]

On définit :
\[
b=f+\frac{K_tK_e}{R}
\]

Équations d’état :
\[
\dot{x}_1=x_2
\]
\[
\dot{x}_2=-\frac{b}{J}x_2+\frac{K_t}{J R}\,u
\]
\[
y=\begin{bmatrix}1&0\end{bmatrix}x
\]

Matrices :
\[
\dot{x}=A x+B u,\qquad y=Cx+Du
\]
\[
A=\begin{bmatrix}
0 & 1\\
0 & -\frac{b}{J}
\end{bmatrix},\quad
B=\begin{bmatrix}
0\\
\frac{K_t}{J R}
\end{bmatrix},\quad
C=\begin{bmatrix}
1 & 0
\end{bmatrix},\quad
D=\begin{bmatrix}
0
\end{bmatrix}
\]

Si ton entrée est directement la PWM :
\[
u=V\approx K_{\mathrm{drv}}\,\mathrm{PWM}\;\Rightarrow\; B_{\mathrm{PWM}}=B\,K_{\mathrm{drv}}
\]

#### 4.9.2 Modèle 3 états (complet, avec \(L\))
États :
\[
x_1=\theta,\quad x_2=\omega,\quad x_3=i,\quad u=V,\quad y=\theta
\]

Équations :
\[
\dot{x}_1=x_2
\]
\[
\dot{x}_2=-\frac{f}{J}x_2+\frac{K_t}{J}x_3
\]
\[
\dot{x}_3=-\frac{K_e}{L}x_2-\frac{R}{L}x_3+\frac{1}{L}u
\]

Matrices :
\[
A=\begin{bmatrix}
0 & 1 & 0\\
0 & -\frac{f}{J} & \frac{K_t}{J}\\
0 & -\frac{K_e}{L} & -\frac{R}{L}
\end{bmatrix},\quad
B=\begin{bmatrix}
0\\
0\\
\frac{1}{L}
\end{bmatrix},\quad
C=\begin{bmatrix}
1 & 0 & 0
\end{bmatrix},\quad
D=\begin{bmatrix}
0
\end{bmatrix}
