# Furuta pendulum with a stepper arm: model, linearisation and LQR design — worked calculation

Prepared for typesetting (LaTeX-ready). Every number below was recomputed independently (numpy, DARE solved by iteration) and matches `analysis/lqr_design.py`.

**Evidence labels used throughout.**
[D] = derivation (pure mathematics). [I] = identified from our own measurements. [S] = simulation only. [H] = measured on hardware. [A] = assumption. Nothing in this document is a successful hardware balance: **the balance controller has not been tested on the hardware.**

---

## 1. Coordinates, signs and units

| Symbol | Meaning | Unit |
|---|---|---|
| $\theta$ | arm angle about the vertical motor axis; positive = clockwise seen from above (the `f` command) [H] | rad |
| $\alpha$ | pendulum angle about the hinge, measured from **hanging down**; positive = the AS5600 count direction | rad |
| $d=\alpha-\pi$ | pendulum angle from **upright**; $d=0$ is the balance point | rad |
| $L_r=0.070$ | motor axis to pendulum hinge (measured, approximate) | m |
| $m_p=0.012$ | pendulum mass (measured) | kg |
| $l_p$ | hinge to pendulum centre of mass (**not yet measured**) | m |
| $J_p$ | pendulum inertia about the hinge | kg m² |
| $b_p$ | equivalent viscous hinge damping | N m s |
| $g=9.81$ | gravity | m/s² |
| $u=\ddot\theta$ | model input: arm angular acceleration | rad/s² |

Stepper: 1/16 microstepping, $3200$ steps per revolution, i.e. $3200/360=8.889$ steps per degree [H, verified: 800 steps $=90^\circ$].

Note: the instructor handout orders the state as $[\alpha,\dot\alpha,\theta,\dot\theta]$ and our code uses $x=[\theta,d,\dot\theta,\dot d]$. The ordering is a bookkeeping choice; the sign conventions above are what matter. [A: the handout's own sign convention for $\theta$ and $\alpha$ was not compared line by line.]

---

## 2. Why the model input is acceleration, not torque

**General (torque) model.** Let the arm have inertia $J_r$ and viscous friction $b_r$ about the motor axis, driven by motor torque $\tau$. The Euler–Lagrange equations for the two coordinates $q=(\theta,\alpha)$ are
$$
\frac{d}{dt}\frac{\partial \mathcal L}{\partial \dot q_i}-\frac{\partial \mathcal L}{\partial q_i}=Q_i,\qquad Q_\theta=\tau-b_r\dot\theta,\quad Q_\alpha=-b_p\dot\alpha .
$$
This gives one **arm equation** (it contains $\tau$, $J_r$, $b_r$) and one **pendulum equation** (it does not contain $\tau$).

**Reduction for a stepper.** A stepper that does not miss steps imposes the arm motion $\theta(t)$ itself. Then $\theta$ is no longer an unknown: $\ddot\theta=u$ is a **prescribed function of time**. The pendulum equation contains $\theta$ only through $\dot\theta,\ddot\theta$, so it can be solved with $u$ as input. The arm equation is no longer needed for the dynamics; it only tells us what torque the motor would have to deliver:
$$
\tau_{\text{required}}=\bigl(J_r+m_pL_r^2+\dots\bigr)u+\text{coupling terms}+b_r\dot\theta .
$$
Therefore $J_r$, $b_r$ and all motor constants **drop out of the design**. They only decide whether the motor can follow the commanded $u$ without missing steps — a hardware limit that is **not verified** (see §10).

---

## 3. Pendulum equation (Lagrange) [D]

Take unit vectors $e_r=(\cos\theta,\sin\theta,0)$, $e_\theta=(-\sin\theta,\cos\theta,0)$, $e_z=(0,0,1)$ (z up). Let the pendulum swing in the plane spanned by $e_\theta$ and $e_z$, with the angle $\alpha$ measured from the downward vertical and positive along $+e_\theta$ (derivation convention). The centre of mass is
$$
\mathbf r=L_r e_r+l_p\sin\alpha\,e_\theta-l_p\cos\alpha\,e_z .
$$
Using $\dot e_r=\dot\theta e_\theta$, $\dot e_\theta=-\dot\theta e_r$:
$$
\dot{\mathbf r}=(L_r\dot\theta+l_p\cos\alpha\,\dot\alpha)\,e_\theta-l_p\sin\alpha\,\dot\theta\,e_r+l_p\sin\alpha\,\dot\alpha\,e_z,
$$
$$
|\dot{\mathbf r}|^2=L_r^2\dot\theta^2+2L_rl_p\cos\alpha\,\dot\theta\dot\alpha+l_p^2\dot\alpha^2+l_p^2\sin^2\alpha\,\dot\theta^2 .
$$
Kinetic energy of the pendulum, for a slender pendulum (inertia about its own axis neglected) with $J_p=I_c+m_pl_p^2$:
$$
T_p=\tfrac12 m_p L_r^2\dot\theta^2+m_pL_rl_p\cos\alpha\,\dot\theta\dot\alpha+\tfrac12 J_p\dot\alpha^2+\tfrac12 J_p\sin^2\alpha\,\dot\theta^2,
\qquad V=-m_pgl_p\cos\alpha .
$$
The $\alpha$ equation:
$$
\frac{d}{dt}\frac{\partial T}{\partial\dot\alpha}=J_p\ddot\alpha+m_pL_rl_p\bigl(\cos\alpha\,\ddot\theta-\sin\alpha\,\dot\theta\dot\alpha\bigr),
$$
$$
\frac{\partial T}{\partial\alpha}=-m_pL_rl_p\sin\alpha\,\dot\theta\dot\alpha+J_p\sin\alpha\cos\alpha\,\dot\theta^2,\qquad
\frac{\partial V}{\partial\alpha}=m_pgl_p\sin\alpha .
$$
The $\dot\theta\dot\alpha$ terms cancel, leaving
$$
\boxed{J_p\ddot\alpha+b_p\dot\alpha+m_pgl_p\sin\alpha+m_pL_rl_p\cos\alpha\,\ddot\theta-J_p\sin\alpha\cos\alpha\,\dot\theta^2=0 .}
$$

**Sign convention.** With $\alpha$ positive along $+e_\theta$ (the direction of arm motion) the coupling term has coefficient $+m_pL_rl_p\cos\alpha$ on the left, i.e. $s=-1$ in the form below. Physically correct: accelerating the hinge forward makes the bob lag *backwards*. If instead the encoder counts positive in the opposite direction, $\alpha\to-\alpha$ flips the sign and $s=+1$. We therefore write
$$
J_p\ddot\alpha+b_p\dot\alpha+m_pgl_p\sin\alpha=s\,m_pl_pL_r\cos\alpha\,\ddot\theta+J_p\sin\alpha\cos\alpha\,\dot\theta^2,\qquad s=\pm1 .
$$
$s$ depends on the direction in which the AS5600 counts increase relative to the arm's forward direction. It is **found from the 1 Hz sine test** (§6), not assumed.

**Centrifugal term.** The term $J_p\sin\alpha\cos\alpha\,\dot\theta^2$ is dropped. Near upright, $\sin\alpha\cos\alpha=\sin d\cos d\approx d$, so the term is $\approx J_pd\,\dot\theta^2$ — a product of the small variable $d$ and $\dot\theta^2$. Compared with the gravity restoring term it modifies the stiffness by $\Delta(\omega_n^2)\sim\dot\theta^2$, so it is negligible when
$$
\dot\theta^2\ll\omega_n^2\approx146\ (\text{rad/s})^2 .
$$
At $\dot\theta=1$ rad/s the ratio is $\dot\theta^2/\omega_n^2=0.7\%$. At the sketch's speed limit ($2\pi$ rad/s) it is $27\%$, so the linear model is **not** valid at that speed. Balancing should stay at small speeds. [A, D]

---

## 4. Normalised form [D]

Divide by $J_p$ and define
$$
\omega_n^2=\frac{m_pgl_p}{J_p},\qquad
2\zeta\omega_n=\frac{b_p}{J_p},\qquad
l_{eq}=\frac{J_p}{m_pl_p}=\frac{g}{\omega_n^2},\qquad
K=\frac{L_r}{l_{eq}} .
$$
Then, with $u=\ddot\theta$ and the centrifugal term neglected,
$$
\boxed{\ddot\alpha+2\zeta\omega_n\dot\alpha+\omega_n^2\sin\alpha=s\,K\cos\alpha\;u .}
$$
Only $\omega_n$, $\zeta$ and $K$ appear. $m_p$ and $l_p$ drop out of the design (they are needed only for the handout's parameter sheet).

**Bound on $l_p$.** Since $J_p=I_c+m_pl_p^2\ge m_pl_p^2$, we have $l_{eq}=J_p/(m_pl_p)\ge l_p$, so $l_p\le l_{eq}$. With the identified $l_{eq}=6.7$ cm and a 7 cm pendulum, the mass must be concentrated near the tip ($l_{eq}=7$ cm would be a point mass at the tip). A uniform rod of length 7 cm would give $l_{eq}=\tfrac23\cdot7=4.7$ cm, so it is not a uniform rod.

---

## 5. Identified numbers [I]

**5.1 Natural frequency.** The free-decay period (arm held, motor off) is $T=0.516$ s (run 1) and $0.523$ s (run 2); we use $T=0.52$ s. The damping is light, so $\omega_d\approx\omega_n$:
$$
\omega_n=\frac{2\pi}{T}=\frac{2\pi}{0.52}=12.083\ \text{rad/s},\qquad \omega_n^2=146.0\ (\text{rad/s})^2 .
$$

**5.2 Equivalent length.**
$$
l_{eq}=\frac{g}{\omega_n^2}=\frac{9.81}{146.0}=0.06719\ \text{m}=6.72\ \text{cm}.
$$

**5.3 Input gain.**
$$
K=\frac{L_r}{l_{eq}}=\frac{0.070}{0.06719}=1.0418 .
$$

**5.4 Damping ratio.** The logarithmic decrement between successive same-side peaks is $\delta=\ln(x_n/x_{n+1})$. Run 1 peaks $-86.1^\circ\to-59.5^\circ$ and $+54.7^\circ\to+36.5^\circ$:
$$
\delta=\ln\frac{86.1}{59.5}=0.370,\qquad \delta=\ln\frac{54.7}{36.5}=0.405 .
$$
Using $\zeta=\delta/\sqrt{4\pi^2+\delta^2}$:
$$
\zeta(0.370)=0.059,\qquad \zeta(0.405)=0.064,\qquad \zeta(0.46)=0.073 .
$$
Later cycles give $\delta=0.58,\,0.69$ ($\zeta=0.09,\,0.11$): the decrement is **not constant**, which a viscous model would require. We use the first-cycle value
$$
\zeta\approx0.06\quad(\text{equivalent viscous; range }0.05\text{–}0.07\text{ on the first cycle}).
$$

**5.5 Dry (Coulomb) friction.** The amplitude falls by an almost constant $\Delta\approx11.4^\circ$ per half cycle. For a pendulum with restoring torque $\approx m_pgl_p\alpha$ (small-angle approximation) and constant friction torque $T_c$, the equilibrium shifts to $\pm T_c/(m_pgl_p)$ and the amplitude drops by $2T_c/(m_pgl_p)$ each half cycle:
$$
\frac{T_c}{m_pgl_p}=\frac{\Delta}{2}=\frac{11.4^\circ\cdot\pi/180}{2}=\frac{0.199}{2}=0.0995\approx0.10 .
$$
With $m_p=0.012$: $T_c=0.10\cdot0.012\cdot9.81\cdot l_p=1.18\times10^{-2}\,l_p$ N m ($l_p$ in m). The formula is a small-angle approximation applied to swings up to $86^\circ$, so it is only indicative.

**5.6 Inertia and damping (need $l_p$).**
$$
J_p=m_pl_pl_{eq}=0.012\cdot0.06719\cdot l_p=8.06\times10^{-4}\,l_p\ \text{kg m}^2,\qquad b_p=2\zeta\omega_nJ_p .
$$
These stay symbolic until $l_p$ is measured (balance the pendulum on an edge). They are **not needed for the LQR design**.

---

## 6. Sign $s$ and model check against the open-loop sine tests [I]

For the hanging pendulum ($\alpha\approx0$, $\sin\alpha\approx\alpha$, $\cos\alpha\approx1$) driven by $\theta=\Theta e^{j\omega t}$, $\ddot\theta=-\omega^2\theta$:
$$
\bigl(\omega_n^2-\omega^2+j\,2\zeta\omega_n\omega\bigr)\alpha=-s\,K\omega^2\theta
\ \Longrightarrow\
\frac{\alpha}{\theta}(j\omega)=\frac{-s\,K\omega^2}{\omega_n^2-\omega^2+j\,2\zeta\omega_n\omega}.
$$
Magnitude:
$$
\left|\frac\alpha\theta\right|=\frac{K\omega^2}{\sqrt{(\omega_n^2-\omega^2)^2+(2\zeta\omega_n\omega)^2}} .
$$

**Meaning of the sign.** Well below resonance ($\omega\ll\omega_n$) the denominator is real and positive, so $\alpha/\theta=-s\cdot(\text{positive})$: for $s=+1$ the pendulum is $180^\circ$ out of phase with the arm; for $s=-1$ it is in phase.

**1 Hz test** ($\omega=6.283$ rad/s): $K\omega^2=1.0418\cdot39.48=41.1$; denominator $146.0-39.5=106.5$ real, imaginary part $2\cdot0.06\cdot12.08\cdot6.283=9.1$, modulus $106.9$:
$$
\left|\frac\alpha\theta\right|=\frac{41.1}{106.9}=0.385,\qquad \text{phase}=180^\circ-\arctan\frac{9.1}{106.5}=175^\circ\ (s=+1).
$$
Measured [H]: ratio $0.31$, phase $\approx156^\circ$. The phase says the pendulum moves opposite to the arm, so **$s=+1$** [I]. The $19^\circ$ phase difference is not explained by the model (encoder nonlinearity, arm angle is the commanded value, dry friction).

**2 Hz test** ($\omega=12.566$ rad/s): $K\omega^2=1.0418\cdot157.9=164.5$; denominator real part $146.0-157.9=-11.9$, imaginary part $2\cdot0.06\cdot12.083\cdot12.566=18.2$, modulus $21.8$:
$$
\left|\frac\alpha\theta\right|=\frac{164.5}{21.8}=7.56,\qquad \text{phase}=180^\circ-\bigl(180^\circ-\arctan\tfrac{18.2}{11.9}\bigr)=57^\circ .
$$
Measured [H]: ratio $4.9$, phase $81^\circ$ at a $40^\circ$ amplitude.

**Correction to an earlier statement.** The predicted phase at 2 Hz is $57^\circ$, not $\approx90^\circ$; $90^\circ$ occurs only exactly at $\omega=\omega_n$. The measurement ($81^\circ$) is closer to resonance than the model places it, which suggests the *effective* $\omega_n$ at that amplitude is closer to $12.6$ rad/s than $12.08$. Near resonance both the ratio and the phase are very sensitive to $\omega_n$ and $\zeta$: for example $\omega_n=12.5$ rad/s (with $K$ recomputed) gives a ratio of $9.3$ and a phase of $85^\circ$, and $\omega_n=12.3$ gives $8.7$ and $70^\circ$.

**Uncertainty of $\omega_n$ [I].** Zero-crossing full periods from the two logs are $0.531,0.523,0.517,0.538$ s (run 1) and $0.556,0.539,0.528,0.526,0.515$ s (run 2), i.e. $\omega_n=11.3$–$12.2$ rad/s, with the period longer at large amplitude. With 10 ms sampling and the $\pm5^\circ$ encoder nonlinearity the uncertainty is about $\pm3\%$. Because $l_{eq}\propto1/\omega_n^2$, that gives $\pm6\%$ in $l_{eq}$ and in $K$. **Re-measure $\omega_n$ at small amplitude ($\le20^\circ$), three times**, before trusting $K$ to better than 10 %. The simulation was run with $K\pm20\%$ (§9).

**Conclusion.** The model captures the right order of magnitude and the sign. The comparison is not a validation; the over-prediction is consistent with dry friction, the $40^\circ$ nonlinear amplitude and the encoder error. [I]

---

## 7. Linearisation about upright [D]

Set $\alpha=\pi+d$. Then $\sin\alpha=-\sin d\approx-d$, $\cos\alpha=-\cos d\approx-1$, and $\dot\alpha=\dot d$. Substituting into the normalised equation:
$$
\ddot d+2\zeta\omega_n\dot d-\omega_n^2d=-s\,K\,u
\ \Longrightarrow\
\boxed{\ddot d=\omega_n^2d-2\zeta\omega_n\dot d-s\,K\,u .}
$$
With $x=[\theta,\ d,\ \dot\theta,\ \dot d]^\top$, $\ddot\theta=u$:
$$
\dot x=Ax+Bu,\quad
A=\begin{bmatrix}0&0&1&0\\0&0&0&1\\0&0&0&0\\0&\omega_n^2&0&-2\zeta\omega_n\end{bmatrix},\quad
B=\begin{bmatrix}0\\0\\1\\-sK\end{bmatrix}.
$$
Numerically ($s=+1$, $\omega_n^2=146.0$, $2\zeta\omega_n=1.450$, $K=1.0418$):
$$
A=\begin{bmatrix}0&0&1&0\\0&0&0&1\\0&0&0&0\\0&146.0&0&-1.450\end{bmatrix},\qquad
B=\begin{bmatrix}0\\0\\1\\-1.0418\end{bmatrix}.
$$
The first two rows are kinematics ($\dot\theta=\dot\theta$, $\dot d=\dot d$). Row 3 says the arm acceleration is the input. Row 4 is the pendulum.

---

## 8. Stability and controllability [D, I]

**Characteristic equation.** The first two rows couple to rows 3–4 only through the lower-right structure; expanding $\det(sI-A)$:
$$
\det(sI-A)=s^2\bigl(s^2+2\zeta\omega_ns-\omega_n^2\bigr)=0 .
$$
Roots: $s=0$ (double, the arm integrator chain) and
$$
s=-\zeta\omega_n\pm\omega_n\sqrt{1+\zeta^2}.
$$
Numerically $\zeta\omega_n=0.725$, $\omega_n\sqrt{1+\zeta^2}=12.083\cdot1.0018=12.105$:
$$
s_1=-0.725+12.105=+11.38,\qquad s_2=-0.725-12.105=-12.83\ \text{rad/s}.
$$
One positive real root: the upright is unstable, with a time constant $1/11.38=88$ ms. That is why the loop must run much faster than this (200 Hz = 5 ms is $\approx$ 18 samples per time constant).

**Controllability matrix.** $C=[B\ \ AB\ \ A^2B\ \ A^3B]$ with
$$
AB=\begin{bmatrix}1\\-K\\0\\2\zeta\omega_nK\end{bmatrix},\quad
A^2B=\begin{bmatrix}0\\2\zeta\omega_nK\\0\\-K\omega_n^2(1+4\zeta^2)\end{bmatrix},\quad
A^3B=\begin{bmatrix}0\\-K\omega_n^2(1+4\zeta^2)\\0\\2\zeta\omega_n^3K(2+4\zeta^2)\end{bmatrix}.
$$
Numerically (columns $B,AB,A^2B,A^3B$; rows $\theta,d,\dot\theta,\dot d$):
$$
C=\begin{bmatrix}
0&1&0&0\\
0&-1.042&1.511&-154.29\\
1&0&0&0\\
-1.042&1.511&-154.29&444.26
\end{bmatrix}.
$$
Expand the determinant along row 1 (its only nonzero entry is the 1 in column 2, cofactor sign $-1$), then along the row of the remaining minor that contains a single 1 (cofactor sign $-1$). The two sign flips cancel, leaving $\det C=\Delta$ with $\Delta$ the $2\times2$ determinant of rows $d,\dot d$ and columns $A^2B,A^3B$:
$$
\Delta=\begin{vmatrix}2\zeta\omega_nK&-K\omega_n^2(1+4\zeta^2)\\-K\omega_n^2(1+4\zeta^2)&2\zeta\omega_n^3K(2+4\zeta^2)\end{vmatrix}
=K^2\omega_n^4\Bigl[4\zeta^2(2+4\zeta^2)-(1+4\zeta^2)^2\Bigr].
$$
The bracket simplifies to $8\zeta^2+16\zeta^4-1-8\zeta^2-16\zeta^4=-1$, so $\Delta=-K^2\omega_n^4$ and
$$
\boxed{\det C=-K^2\omega_n^4=-(1.0418)^2(146.0)^2=-2.31\times10^4\neq0 .}
$$
The rank is 4 for every $\zeta$, as long as $K\ne0$ and $\omega_n\ne0$. The system is controllable [D]; the numerical rank computed by the script is also 4.

---

## 9. Discrete LQR design [D, S]

**Cost.** Minimise $\displaystyle J=\sum_{k=0}^{\infty}\bigl(x_k^\top Qx_k+u_k^\top Ru_k\bigr)$ subject to $x_{k+1}=A_dx_k+B_du_k$.

**Discretisation (zero-order hold, $\Delta t=5$ ms).**
$$
\begin{bmatrix}A_d&B_d\\0&1\end{bmatrix}=\exp\!\left(\begin{bmatrix}A&B\\0&0\end{bmatrix}\Delta t\right).
$$

**Weights (Bryson's rule: $1/(\text{acceptable value})^2$).**
$$
Q=\operatorname{diag}\!\left(\tfrac1{0.5^2},\tfrac1{0.08^2},\tfrac1{3^2},\tfrac1{5^2}\right)=\operatorname{diag}(4,\ 156.25,\ 0.1111,\ 0.04),\qquad
R=\tfrac1{15^2}=4.444\times10^{-3},
$$
that is $0.5$ rad of arm drift, $0.08$ rad ($4.6^\circ$) pendulum error, $3$ rad/s arm speed, $5$ rad/s pendulum speed, $15$ rad/s² acceleration. These are design choices [A]: tighter weights balanced in simulation but chattered because of encoder quantisation and derivative noise.

**Method (Riccati).** Solve the discrete algebraic Riccati equation
$$
P=A_d^\top PA_d-A_d^\top PB_d\,(R+B_d^\top PB_d)^{-1}B_d^\top PA_d+Q,
$$
and form the gain
$$
K_d=(R+B_d^\top PB_d)^{-1}B_d^\top PA_d,\qquad u_k=-K_dx_k .
$$
It was solved by fixed-point iteration (and by `scipy.linalg.solve_discrete_are` in the script). Both give
$$
\boxed{K_d=\bigl[-27.555,\ -600.497,\ -14.217,\ -44.909\bigr],\qquad u=-\bigl(k_1\theta+k_2d+k_3\dot\theta+k_4\dot d\bigr).}
$$
The closed-loop discrete eigenvalues have moduli $0.9308$ (pair) and $0.9833$ (pair), all $<1$; as continuous poles $\ln\lambda/\Delta t$: $-14.3\pm6.9j$ and $-3.4\pm2.4j$ rad/s. The fast pair is faster than the open-loop unstable pole ($11.4$ rad/s), as it must be.

**Sanity check 1 — signs.** $k_2<0$ means $u=-k_2d=+600\,d$: for $d>0$ the arm accelerates in $+\theta$. From $\ddot d=\omega_n^2d-sKu$ with $s=+1$ this makes $\ddot d$ more negative and pushes $d$ back to zero — the right correction direction. If $s$ were $-1$ the same gains would destabilise the loop, which is why $s$ must be right.

**Sanity check 2 — pendulum alone.** Keep only $k_2,k_4$ (ignore the arm states). Then
$$
\ddot d=(\omega_n^2+Kk_2)\,d+(-2\zeta\omega_n+Kk_4)\,\dot d=-479.6\,d-48.24\,\dot d,
$$
with roots $-14.0$ and $-34.2$ rad/s. Stability requires $\omega_n^2+Kk_2<0$, i.e. $|k_2|>\omega_n^2/K=146.0/1.0418=140.1$; our $600.5$ is $4.3$ times that limit. The $-14.0$ rad/s root matches the fast closed-loop pole above.

**Sanity check 3 — static (steady-state) condition.** In steady state $u=0$, $\ddot\theta=0$, $\dot\theta=0$, $\dot d=0$. Row 4 then forces $d=0$. If the pendulum angle used by the controller has a calibration bias $b$ ($d_{meas}=d+b$), $u=0$ requires $k_1\theta+k_2b=0$:
$$
\theta_{ss}=-\frac{k_2}{k_1}\,b=-\frac{-600.497}{-27.555}\,b=-21.8\,b .
$$
A $1^\circ$ bias therefore makes the arm sit $\approx22^\circ$ off centre, matching the simulation ($-21.7^\circ$). The sketch's slow auto-trim $\dot c=-k_i\theta$ ($k_i=0.023$) integrates this offset away; it is an addition to the LQR, outside the LQR optimality proof [S].

**Simulation results [S, not hardware].** A nonlinear simulation (5° start, quantised encoder, filtered derivative, 1-sample delay, $|u|\le100$ rad/s²) balances for the nominal plant and with $K\times0.8$ and $K\times1.2$; with a $1^\circ$ bias the trim removes the $22^\circ$ arm offset. It does **not** include dry hinge friction, the $\pm5^\circ$ encoder nonlinearity or motor torque limits.

---

## 10. Implementation notes and what they assume

- **Speed command.** The controller integrates $u$ to a speed command, $\dot\theta\leftarrow\dot\theta+u\,\Delta t$, and uses the commanded speed as $\dot\theta$ in the feedback. This assumes the stepper really follows the commanded speed.
- **Derivative.** $\dot d$ comes from a difference of the encoder angle through a 25 Hz low-pass ($\tau=1/(2\pi\cdot25)=6.4$ ms). The filter delay is not in the LQR design.
- **Untested motor limits.** The arm test (§ motion_core) only reached $60^\circ$/s ($533$ steps/s). The sketch allows $2\pi$ rad/s $=3200$ steps/s and $|u|\le100$ rad/s² $=5.1\times10^4$ steps/s². A stepper at 12 V loses torque at high step rates, so **missed steps at these limits are likely and undetectable** (no arm encoder).

---

## 11. Assumptions and limitations (summary)

1. **No arm encoder.** $\theta,\dot\theta$ are the *commanded* values. Missed steps cannot be detected; the 90° test only verifies small slow moves. [H]
2. **Dry hinge friction.** About $10\%$ of the maximum gravity torque; the pendulum stops abruptly and rests off-centre. The viscous model with $\zeta\approx0.06$ is only an equivalent. Dry friction adds a dead band near upright.
3. **Encoder nonlinearity** about $\pm5^\circ$ (weak magnet). It limits the accuracy of $\omega_n$, $\zeta$ and the zero, and it is the same order as the controller's workspace ($\pm4^\circ$ catch window).
4. **Identified parameters are uncertain:** $\omega_n\pm3\%$, hence $K\pm6\%$; $L_r$ is approximate; $l_p$, $J_p$, $b_p$ are not measured.
5. **Sign conventions.** $s=+1$ was read from the 1 Hz phase ($156^\circ$ vs predicted $175^\circ$). The user has **not yet chosen** the final direction convention for $\theta$; if the encoder direction or the arm direction is changed, $s$ must be re-derived.
6. **Linear model validity.** Needs small $d$ and $\dot\theta^2\ll\omega_n^2$ (§3). No swing-up is covered here.
7. **Delay and filtering** are not in the LQR design (only in the simulation).
8. **Not verified on hardware:** balancing itself, the motor's ability to deliver $u$, the gains' robustness to friction. Treat all controller results as simulation until a measured log shows otherwise.

---

## 12. Next steps (evidence required)

1. Re-measure $\omega_n$ at $\le20^\circ$, three runs; recompute $l_{eq}$, $K$.
2. Measure $l_p$ (edge balance) to complete $J_p$, $b_p$, $T_c$.
3. Choose and record the $\theta$ direction convention; recheck $s$.
4. Hardware test of the balance controller with bounded limits (small $V_{max}$, $U_{max}$) and the emergency stop; log $\theta$, $d$, $u$, trim at 100 Hz; compare with the simulation.
