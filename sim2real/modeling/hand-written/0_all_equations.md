# Gimbal hand model: all equations, step by step

**How to read this file.** Every step has the same layout:

1. **Where we start, what we want, what we get** (known inputs, unknowns, what is set to 0, the pass rule)
2. **The equations**, numbered, each followed by a table of its symbols with a **Status** column
3. **Plug in, get out** (the data flow of the step)
4. **Conclusion** (what you hold at the end and how it is checked)

Status words used in the tables:

| Status | Meaning |
|---|---|
| **known input** | a number we choose or measure (foundational), we plug it in |
| **unknown** | what the step solves for |
| **output** | computed by an equation in this step |
| **set to 0** | switched off in this step |
| **NEW** | first appears in this step |

**Test case for every step:** rod released at rest from a **5 degree** tilt toward **45 degrees** (halfway between the +x and +y directions), unless the step says otherwise.

**Integrator:** RK4 (fixed step, $\Delta t=0.001$ s), the same method MuJoCo uses in this model.

**Two meanings of g.** Plain $g=9.81$ m/s² is gravity. Bold $\mathbf g(q)$ is the vector of gravity torques on the two hinges.

---

## All symbols in one place

| Symbol | Meaning | Unit | Code name |
|---|---|---|---|
| $m$ | rod mass | kg | `rod_mass` |
| $L$ | rod length, pivot to tip | m | `rod_length` |
| $r$ | rod radius | m | `rod_radius` |
| $g$ | gravity | m/s² | `gravity` |
| $\Delta t$ | time step | s | `time_step` |
| $I_p$ | inertia about the pivot, across the rod, $mL^2/3$ | kg·m² | `pivot_inertia` |
| $I_{zz}$ | inertia about the rod's own long axis, $mr^2/2$ | kg·m² | `axial_inertia` |
| $U_0$ | gravity energy scale, $mgL/2$ | J | `gravity_energy_scale` |
| $q_1$ | outer hinge angle (about the fixed x axis) | rad | `outer_angle` |
| $q_2$ | inner hinge angle (about the hinge carried by the outer ring) | rad | `inner_angle` |
| $\dot q_1,\ \dot q_2$ | hinge rates | rad/s | `outer_rate`, `inner_rate` |
| $\ddot q_1,\ \ddot q_2$ | hinge accelerations | rad/s² | `angular_acceleration` |
| $M_{11}$ | effective inertia seen by the outer hinge | kg·m² | `outer_inertia` |
| $\mathbf h$ | velocity coupling torques (figure skater and swing out effects) | N·m | `velocity_bias_torque` |
| $\mathbf g(q)$ | gravity torques on the hinges | N·m | `gravity_torque` |
| $\mathbf u$ | motor torques after the clip | N·m | `motor_torque` |
| $b$ | viscous damping coefficient | N·m·s/rad | `viscous_damping` |
| $c$ | Coulomb friction level | N·m | `coulomb_friction` |
| $\varepsilon$ | smoothing width of the friction sign | rad/s | `friction_smoothing_width` |
| $y$ | state $(q_1,q_2,\dot q_1,\dot q_2)$ | mixed | `state` |
| $E$ | total energy | J | `total_energy` |
| $k_p,\ k_d$ | PD gains | N·m/rad, N·m·s/rad | `proportional_gain`, `derivative_gain` |
| $\sigma$ | angle measurement noise std | rad | `noise_std` |
| $\eta$ | one noise sample | rad | `noise_samples` |

---

## What is ON, OFF or NEW in each step

| Term | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|
| Gravity torque $\mathbf g(q)$ | on | on | on | on | on | on | on | on | on |
| Velocity coupling $\mathbf h$ | on | on | on | on | on | on | on | on | on |
| Motor torque $\mathbf u$ | **0** | **0** | **0** | **0** | **NEW**: constant command, clipped | **NEW**: $\mathbf g(q_0)$ | **NEW**: PD | **NEW**: computed torque | PD run is the data source |
| Viscous $b$ | **0** | **NEW** 0.04 | 0.04 | 0, 0.04, 0.04 (A, B, C) | 0 and 0.04 (two runs) | 0 and 0.04 | 0 and 0.04 | 0 and 0.04 | 0.04 |
| Coulomb $c$ | **0** | **0** | **NEW** 0.02 | 0, 0, 0.02 (A, B, C) | 0 and 0.02 | 0 and 0.02 | 0 and 0.02 | 0 and 0.02 | 0.02 |
| Measurement noise $\eta$ | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | **NEW** |
| Compared with MuJoCo | yes | yes | yes | yes | yes | yes | yes | yes | no (noise check) |

---

# Step 1 (gimbal phase 1): Stage A, the frictionless free fall

## 1. Where we start, what we want, what we get

| | |
|---|---|
| **We have (known inputs)** | $m,\ L,\ r,\ g,\ \Delta t$; start tilt $\theta_0=5^\circ$; lean direction $\varphi_0=45^\circ$; start at rest |
| **Unknown** | the motion after release: $q_1(t),\ q_2(t),\ \dot q_1(t),\ \dot q_2(t)$ |
| **Set to 0 (switched off)** | motor torque $\mathbf u=0$, viscous damping $b=0$, Coulomb friction $c=0$, noise $=0$ |
| **What we do** | write the equations of motion, turn them into $y'=f(y)$, step forward with RK4 |
| **What we get** | arrays of angles, rates and energy after each step, until the tip passes horizontal |
| **Pass rule** | MuJoCo (frictionless, same start): max abs angle error $\le 10^{-6}$ rad |

## 2. The equations

**1.1 Constants** (derived from the known inputs)

$$I_p=\frac{mL^2}{3},\qquad I_{zz}=\frac{mr^2}{2},\qquad U_0=\frac{mgL}{2}$$

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $m, L, r, g$ | mass, length, radius, gravity | kg, m, m, m/s² | known input | `rod_mass`, `rod_length`, `rod_radius`, `gravity` |
| $I_p$ | inertia about the pivot | kg·m² | output (1/3) | `pivot_inertia` |
| $I_{zz}$ | inertia about the long axis | kg·m² | output (2e-4) | `axial_inertia` |
| $U_0$ | gravity energy scale | J | output (4.905) | `gravity_energy_scale` |

**1.2 Start angles from tilt and direction.** The rod axis is $u=(\sin q_2,\ -\sin q_1\cos q_2,\ \cos q_1\cos q_2)$. It must equal $(\sin\theta_0\cos\varphi_0,\ \sin\theta_0\sin\varphi_0,\ \cos\theta_0)$. Solving:

$$q_2=\arcsin(\sin\theta_0\cos\varphi_0),\qquad q_1=\arcsin\!\left(\frac{-\sin\theta_0\sin\varphi_0}{\cos q_2}\right)$$

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $\theta_0,\ \varphi_0$ | start tilt, lean direction | rad | known input (5°, 45°) | `START_TILT_DEG`, `LEAN_DIRECTION_DEG` |
| $q_1,\ q_2$ at $t=0$ | start angles | rad | output | `start_angles` |
| $\dot q_1,\ \dot q_2$ at $t=0$ | start rates | rad/s | known input (0, 0) | `start_rates` |

**1.3 Tip height** (used only to stop the run when the rod passes horizontal)

$$z_{\text{tip}}=L\cos q_1\cos q_2$$

**1.4 Mass matrix.** It is diagonal because the two hinge axes are perpendicular to each other and the rod is symmetric.

$$M(q)=\begin{pmatrix}M_{11}&0\\0&I_p\end{pmatrix},\qquad M_{11}=I_p\cos^2 q_2+I_{zz}\sin^2 q_2$$

**1.5 Velocity coupling torques** (zero when the rod is at rest)

$$h_1=(I_{zz}-I_p)\sin 2q_2\ \dot q_1\dot q_2,\qquad h_2=-\tfrac12(I_{zz}-I_p)\sin 2q_2\ \dot q_1^{2}$$

**1.6 Gravity torques.** Potential energy is $U=U_0\cos q_1\cos q_2$ (zero at pivot height). The gravity torque vector is its gradient:

$$g_1=\frac{\partial U}{\partial q_1}=-U_0\sin q_1\cos q_2,\qquad g_2=\frac{\partial U}{\partial q_2}=-U_0\cos q_1\sin q_2$$

**1.7 Equation of motion** (Euler-Lagrange, derived in E1 to E4)

$$M(q)\,\ddot q+\mathbf h(q,\dot q)+\mathbf g(q)=\boldsymbol\tau,\qquad \boldsymbol\tau=\mathbf u-b\,\dot q-c\,s(\dot q)$$

In step 1: $\mathbf u=0,\ b=0,\ c=0$, so $\boldsymbol\tau=0$. Because $M$ is diagonal, solving for the accelerations is two divisions:

$$\ddot q_i=\frac{\tau_i-h_i-g_i}{M_{ii}},\qquad i=1,2$$

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $\ddot q_i$ | hinge acceleration | rad/s² | output (this is what the equation gives us) | `angular_acceleration` |
| $\boldsymbol\tau$ | total torque on the hinges | N·m | **set to 0** | `motor_torque` (zeros placeholder) |
| $\mathbf h,\ \mathbf g$ | from 1.5, 1.6 | N·m | output | `velocity_bias_torque`, `gravity_torque` |

**1.8 State form.** RK4 integrates a first-order system, so we stack angles and rates:

$$y=\begin{pmatrix}q_1\\q_2\\\dot q_1\\\dot q_2\end{pmatrix},\qquad y'=f(y,\boldsymbol\tau)=\begin{pmatrix}\dot q_1\\\dot q_2\\\ddot q_1\\\ddot q_2\end{pmatrix}$$

**1.9 RK4 step** (the torque $\boldsymbol\tau$ is held constant across the four slope samples)

$$k_1=f(y_k),\quad k_2=f\!\left(y_k+\tfrac{\Delta t}{2}k_1\right),\quad k_3=f\!\left(y_k+\tfrac{\Delta t}{2}k_2\right),\quad k_4=f(y_k+\Delta t\,k_3)$$

$$y_{k+1}=y_k+\frac{\Delta t}{6}\,(k_1+2k_2+2k_3+k_4)$$

**1.10 Energy** (conserved when $\boldsymbol\tau=0$)

$$E=\tfrac12\left(M_{11}\dot q_1^{2}+I_p\dot q_2^{2}\right)+U_0\cos q_1\cos q_2$$

## 3. Plug in, get out

```
known inputs (m, L, r, g, dt, 5 deg, 45 deg)
   -> 1.1 constants (I_p, I_zz, U0)
   -> 1.2 start state y0 = (q1, q2, 0, 0)
   -> repeat: 1.4 M, 1.5 h, 1.6 g -> 1.7 q_ddot -> 1.8 f(y) -> 1.9 RK4 -> y_(k+1)
   -> store y_(k+1) and E  (row k is the state at t = (k+1) dt)
   -> stop when z_tip < 0
```

## 4. Conclusion

At the end you hold the frictionless trajectory of the rod as arrays. Checks:

| Check | Expected |
|---|---|
| $M_{11}$ at $q_2=0.8$ | 0.161903 kg·m² |
| $\mathbf h+\mathbf g$ at $q=(0.5,0.8),\ \dot q=(1,2)$ | $(-2.30435,\ -2.92139)$ N·m, equal to MuJoCo `qfrc_bias` |
| start angles | $(-0.0618,\ +0.0617)$ rad |
| first acceleration at rest | $(-0.910,\ +0.905)$ rad/s² |
| max angle error vs MuJoCo | $\le 10^{-6}$ rad |
| energy drift in the fall | tiny (my test run: about $10^{-10}$ J) |


---

# Step 2 (gimbal phase 1): Stage B, add viscous damping

## 1. Where we start, what we want, what we get

| | |
|---|---|
| **We have** | the full stage A model (steps 1.1 to 1.10), now plus a damping coefficient $b=0.04$ N·m·s/rad on both hinges |
| **Unknown** | the same motion $q(t),\ \dot q(t)$, now slowed by damping |
| **Set to 0 (still)** | motor torque $\mathbf u=0$, Coulomb friction $c=0$, noise $=0$ |
| **NEW in this step** | the viscous torque $-b\dot q$ inside $\boldsymbol\tau$; the energy balance |
| **What we get** | the damped trajectory; energy that only decreases, by exactly the work done against damping |
| **Pass rule** | MuJoCo with `damping = 0.04`, `frictionloss = 0`: max abs angle error $\le 10^{-6}$ rad |

## 2. The equations

Everything from step 1 is unchanged (1.1 to 1.10) except the total torque.

**2.1 Viscous torque.** A resistance proportional to the hinge speed, always opposing the motion (like a bearing in oil):

$$\tau^{\text{visc}}_i=-b\,\dot q_i$$

**2.2 Total torque and acceleration**

$$\boldsymbol\tau=\mathbf u-b\,\dot q-c\,s(\dot q),\qquad \mathbf u=0,\ c=0\ \Rightarrow\ \tau_i=-b\,\dot q_i$$

$$\ddot q_i=\frac{-b\,\dot q_i-h_i-g_i}{M_{ii}}$$

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $b$ | viscous damping coefficient | N·m·s/rad | **NEW**, known input (0.04) | `viscous_damping` |
| $\tau^{\text{visc}}_i$ | damping torque on hinge $i$ | N·m | **NEW**, output | `viscous_torque` |
| $\ddot q_i$ | acceleration | rad/s² | output | `angular_acceleration` |
| $\mathbf u,\ c$ | motor torque, Coulomb level | N·m | **set to 0** | |

**2.3 Energy balance.** Differentiate $E$ along the motion: $\dot E=\dot q^{\mathsf T}\boldsymbol\tau$. With only damping:

$$\dot E=-b\left(\dot q_1^{2}+\dot q_2^{2}\right)\le 0\qquad\Rightarrow\qquad E_k-E_0\approx-\int_0^{t_k} b\left(\dot q_1^{2}+\dot q_2^{2}\right)dt$$

The integral is computed from the logged rates with the trapezoid rule. This is a check that uses only the hand data.

**2.4 Growth rate with damping (linear theory near upright).** Small angles give $I_p\ddot q=U_0\,q-b\,\dot q$. Trying $q\propto e^{st}$:

$$I_p s^2+b\,s-U_0=0\ \Rightarrow\ s=\frac{-\beta+\sqrt{\beta^2+4p}}{2},\qquad \beta=\frac{b}{I_p},\ p=\frac{U_0}{I_p}$$

With $\beta=0.12$ 1/s and $p=14.715$ 1/s²: $s=3.776$ 1/s (undamped: 3.836 1/s). Damping slows the fall by about 1.6 percent and can never stop it.

## 3. Plug in, get out

```
step 1 model + b = 0.04
   -> 2.1 viscous torque -> 2.2 total torque -> q_ddot -> RK4 (unchanged) -> next state
   -> log state and energy
   -> check: energy loss = integral of b |q_dot|^2 dt   (2.3)
   -> compare with MuJoCo (damping 0.04, no frictionloss)
```

## 4. Conclusion

You hold the damped trajectory. It must match MuJoCo to $10^{-6}$ rad, and the energy lost along the way must equal the damping work (2.3). Damping changes the timing of the fall by a small amount, not its nature.


---

# Step 3 (gimbal phase 1): Stage C, add Coulomb friction

## 1. Where we start, what we want, what we get

| | |
|---|---|
| **We have** | the stage B model, plus a dry friction level $c=0.02$ N·m on both hinges |
| **Unknown** | the same motion, now also resisted by a constant-size friction torque |
| **Set to 0 (still)** | motor torque $\mathbf u=0$, noise $=0$ |
| **NEW in this step** | $\tau^{\text{coul}}_i=-c\tanh(\dot q_i/\varepsilon)$, the smoothing width $\varepsilon$ |
| **What we get** | the full passive model: gravity, coupling, viscous and Coulomb friction |
| **Pass rule** | MuJoCo with `damping = 0.04`, `frictionloss = 0.02`: max abs angle error $\le 10^{-3}$ rad. Looser than steps 1 and 2 because the two simulators use different friction models (see 3.2) |

## 2. The equations

**3.1 Ideal Coulomb friction.** A constant-size torque opposing the motion:

$$\tau^{\text{coul}}_i=-c\,\operatorname{sgn}(\dot q_i)$$

The sign jumps from $-1$ to $+1$ at $\dot q=0$. A numerical integrator near zero speed overshoots across the jump and chatters. The size of the chatter is the speed change friction can cause in one step: $(c/I_p)\,\Delta t=0.06\times0.001=6\times10^{-5}$ rad/s.

**3.2 Smoothed version we use**

$$\tau^{\text{coul}}_i=-c\,\tanh\!\left(\frac{\dot q_i}{\varepsilon}\right),\qquad \varepsilon=10^{-3}\ \text{rad/s}$$

For $|\dot q|\gg\varepsilon$ this equals $-c\operatorname{sgn}(\dot q)$, and it passes smoothly through zero. MuJoCo's `frictionloss` is instead a constraint that can hold the joint exactly still. This difference is the reason for the $10^{-3}$ rad tolerance.

**3.3 Choosing $\varepsilon$.** Two pressures:

| $\varepsilon$ too small | $\varepsilon$ too large |
|---|---|
| slope $c/\varepsilon$ is huge and RK4 goes unstable; stable while $\Delta t\,(c/\varepsilon)/I_p\lesssim 2.8$ | friction fades away at low speed |

At $\varepsilon=10^{-3}$: $\Delta t\,(c/\varepsilon)/I_p=0.06$, safely stable, and $\varepsilon$ is about 16 times the chatter size.

**3.4 Total torque and acceleration**

$$\tau_i=-b\,\dot q_i-c\tanh\!\left(\frac{\dot q_i}{\varepsilon}\right)\quad(\mathbf u=0),\qquad \ddot q_i=\frac{\tau_i-h_i-g_i}{M_{ii}}$$

**3.5 Breakaway tilt.** At rest, friction can hold the rod while gravity torque is below $c$. Near upright the gravity torque is $U_0\sin\theta$, so the rod is stuck for

$$\theta<\theta_{\text{break}}=\arcsin\!\left(\frac{c}{U_0}\right)=0.23^\circ$$

In the model with $\tanh$ the rod does not stick perfectly: it creeps at the rate where friction balances gravity, $\dot q=\varepsilon\,\operatorname{artanh}(\text{gravity torque}/c)$. The notebook shows this creep next to MuJoCo's exact sticking for a $0.1^\circ$ start.

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $c$ | Coulomb friction level | N·m | **NEW**, known input (0.02) | `coulomb_friction` |
| $\varepsilon$ | smoothing width | rad/s | **NEW**, known input (1e-3) | `friction_smoothing_width` |
| $\tau^{\text{coul}}_i$ | friction torque | N·m | **NEW**, output | `coulomb_torque` |
| $\theta_{\text{break}}$ | breakaway tilt | deg | output (0.23) | `breakaway_tilt_deg` |

## 3. Plug in, get out

```
step 2 model + c = 0.02, eps = 1e-3
   -> 3.2 smooth Coulomb torque + 2.1 viscous torque -> 3.4 total torque -> q_ddot -> RK4 -> next state
   -> compare with MuJoCo (damping 0.04, frictionloss 0.02) at 1e-3 rad
   -> extra case: 0.1 degree start, hand creeps, MuJoCo sticks
```

## 4. Conclusion

You hold the full passive model: the rod falls under gravity with coupling, viscous damping and smoothed dry friction. Phase 1 is complete once step 4 overlays all three stages.


---

# Step 4 (gimbal phase 1): Overlay on the clean MuJoCo runs

## 1. Where we start, what we want, what we get

| | |
|---|---|
| **We have** | the three hand models: A (frictionless), B (viscous only), C (viscous and Coulomb), all from the 5 degree, 45 degree start; the MuJoCo runs with the same settings |
| **Unknown** | none, this is a comparison step |
| **Set to 0** | motor torque, noise (as before) |
| **NEW in this step** | the row alignment rule and the error metric |
| **What we get** | one figure and one table: hand against MuJoCo for A, B and C, plus a demonstration of why alignment matters |
| **Pass rule** | A and B: $10^{-6}$ rad; C: $10^{-3}$ rad (fixed before looking) |

## 2. The equations

**4.1 Row alignment.** In MuJoCo (and in the hand simulator) each log row is saved **after** the step. Row $k$ (counting from 0) is the state at

$$t_k=(k+1)\,\Delta t$$

We compare index to index: hand row $k$ with MuJoCo row $k$. Saving the initial state as a hand row would shift everything by one step and create a false error of about $\dot q\,\Delta t$.

**4.2 Error metric**

$$e_{k,i}=\left|q^{\text{hand}}_{k,i}-q^{\text{MuJoCo}}_{k,i}\right|,\qquad e_{\max}=\max_{k,i}e_{k,i}$$

**4.3 Pass rule**

$$e_{\max}\le\text{tolerance}\qquad(10^{-6}\ \text{rad for A and B},\ 10^{-3}\ \text{rad for C})$$

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $k$ | log row index (0 based) | 1 | known input | `row_index` |
| $t_k$ | time label of row $k$ | s | output | `time_axis` |
| $e_{k,i}$ | angle error | rad | output | `angle_error` |
| tolerance | pass level | rad | known input, fixed before looking | `TOLERANCE_RAD` |

## 3. Plug in, get out

```
hand A, B, C runs (steps 1 to 3)  +  MuJoCo A, B, C runs
   -> align index to index (4.1)
   -> 4.2 error arrays -> 4.3 compare with tolerance
   -> figure (overlay + error) and table
   -> extra: shift hand rows by one and show the false error
```

## 4. Conclusion

Phase 1 passes if all three rows of the table say PASS. At that point the hand model of the passive gimbal is trusted, and the actuator is added on top of it in phase 2.


---

# Step 5 (gimbal phase 2): Motor torque, open loop

## 1. Where we start, what we want, what we get

| | |
|---|---|
| **We have** | the full passive model from phase 1, plus two torque motors (one per hinge): gear 1, command limit $\pm3$ N·m |
| **Unknown** | the motion under a torque that we choose in advance (no feedback) |
| **Set to 0 (still)** | noise $=0$. Friction is run both off and on (two runs) |
| **NEW in this step** | the motor torque $\mathbf u$ in $\boldsymbol\tau$: command, clip, gear |
| **What we get** | trajectories under constant commands, including one that saturates |
| **Pass rule** | frictionless runs: $10^{-6}$ rad; run with damping and friction: $10^{-3}$ rad |

## 2. The equations

**5.1 Motor torque.** The command $\mathbf c_{\text{cmd}}$ is a number we choose for each motor. The motor clips it to its limit, and the gear multiplies it:

$$u_i=G\cdot\operatorname{clip}\!\left(c_{\text{cmd},i},\,-\tau_{\max},\,+\tau_{\max}\right)$$

**5.2 The torque is a generalized force on its own hinge.** The motor pushes about the hinge axis, so its virtual work is $u_i\,\delta q_i$ and it enters the right-hand side of the equation of motion as $u_i$ in row $i$. There is no cross-coupling because the inner hinge axis is perpendicular to the outer one.

**5.3 Equation of motion with the motor**

$$M(q)\,\ddot q+\mathbf h+\mathbf g=\mathbf u-b\,\dot q-c\tanh\!\left(\frac{\dot q}{\varepsilon}\right)\ \Rightarrow\ \ddot q_i=\frac{u_i-b\dot q_i-c\tanh(\dot q_i/\varepsilon)-h_i-g_i}{M_{ii}}$$

**5.4 The torque is held constant over one step.** MuJoCo sets the command once per step. So in RK4 the same $\mathbf u$ is used in all four stages $k_1$ to $k_4$ (equations 1.9 unchanged).

**5.5 First acceleration (hand check).** At rest, $\mathbf h=0$ and friction is zero, so

$$\ddot q_i(0)=\frac{u_i-g_i(q_0)}{M_{ii}(q_0)}$$

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $c_{\text{cmd},i}$ | motor command | N·m | **NEW**, known input (we choose) | `motor_command` |
| $G$ | gear ratio | 1 | **NEW**, known input (1) | `motor_gear` |
| $\tau_{\max}$ | command limit | N·m | **NEW**, known input (3) | `motor_torque_limit` |
| $u_i$ | motor torque on hinge $i$ | N·m | **NEW**, output | `motor_torque` |

**Cases run in the notebook**

| Case | Command (N·m) | Friction | Tolerance |
|---|---|---|---|
| a | $(+0.5,\ -0.5)$, for 1.0 s | off | $10^{-6}$ rad |
| b | $(+5,\ -5)$, clipped to $(+3,\ -3)$, for 0.5 s | off | $10^{-6}$ rad |
| c | $(+0.5,\ -0.5)$, for 1.0 s | $b=0.04$, $c=0.02$ | $10^{-3}$ rad |

## 3. Plug in, get out

```
command (we choose) -> 5.1 clip and gear -> u
   -> 5.3 q_ddot (with u constant over the step) -> RK4 -> next state
   -> compare with MuJoCo (same constant ctrl, same friction settings)
```

## 4. Conclusion

You hold a simulator that responds to torque and saturates like the real motors. The saturation case is the check that the clip is right: the logged torque must read $\pm3$ N·m when the command is $\pm5$.


---

# Step 6 (gimbal phase 2): Hold the rod with the gravity torque, open loop

## 1. Where we start, what we want, what we get

| | |
|---|---|
| **We have** | the step 5 simulator, the start state $q_0$ (5 degrees, 45 degrees), at rest |
| **Unknown** | the torque that keeps the rod still, and whether the hold is stable |
| **Set to 0** | noise $=0$; friction off for the exact-equilibrium runs |
| **NEW in this step** | the hold torque $\mathbf u=\mathbf g(q_0)$ and the hold limit |
| **What we get** | the rod stays at $q_0$ in both simulators; with a small push it leaves exponentially |
| **Pass rule** | exact start: drift $\le 10^{-8}$ rad for 3 s in hand and MuJoCo; perturbed start: hand vs MuJoCo $\le 10^{-6}$ rad |

## 2. The equations

**6.1 Hold torque.** At rest, $\dot q=0$ so $\mathbf h=0$. The equation of motion gives $\ddot q=0$ exactly when the motor cancels gravity:

$$\mathbf u_{\text{hold}}=\mathbf g(q_0)\ \Rightarrow\ \ddot q=\frac{\mathbf u_{\text{hold}}-\mathbf g(q_0)}{M}=0$$

At the 5 degree start this is about $(+0.302,\ -0.302)$ N·m.

**6.2 This equilibrium is unstable.** A small deviation $\delta=q-q_0$ feels $\mathbf g(q)-\mathbf g(q_0)\approx\frac{\partial\mathbf g}{\partial q}\delta$ with a negative entry on the diagonal, so $M\ddot\delta=-\frac{\partial\mathbf g}{\partial q}\delta$ grows like $e^{st}$ with

$$s\approx\sqrt{\frac{U_0}{I_p}}=3.836\ \text{1/s}$$

A deviation of $10^{-3}$ rad grows by about $e^{3.8}\approx45$ in one second. The notebook starts with $\delta_0=(10^{-3},0)$ and fits the slope of $\ln\lVert\delta\rVert$.

**6.3 Hold limit (single axis).** The motor can give at most $\tau_{\max}$ and gravity torque at tilt $\theta$ is $U_0\sin\theta$:

$$\theta_{\text{hold}}=\arcsin\!\left(\frac{\tau_{\max}}{U_0}\right)=37.7^\circ$$

Necessary, not sufficient: it says the motor is strong enough to cancel gravity statically, not that the hold is stable.

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $\mathbf u_{\text{hold}}$ | hold torque | N·m | **NEW**, output | `hold_torque` |
| $q_0$ | start angles (the hold target) | rad | known input | `start_angles` |
| $\delta$ | deviation from $q_0$ | rad | output | `deviation` |
| $s$ | exponential growth rate of $\delta$ | 1/s | output (fit) | `fitted_growth_rate` |
| $\theta_{\text{hold}}$ | hold limit | deg | output (37.7) | `hold_limit_deg` |

## 3. Plug in, get out

```
start angles q0 -> 6.1 hold torque u = g(q0) (a constant, computed once)
   -> run step 5 simulator with that constant command
   -> case a: exact start, deviation stays ~0
   -> case b: start + 1e-3 rad, deviation grows like exp(s t), fit s
   -> compare with MuJoCo
```

## 4. Conclusion

The rod stays put because the motor cancels gravity at exactly one configuration, and any error is amplified at 3.8 per second. This is why a constant torque is not a controller: step 7 adds feedback.


---

# Step 7 (gimbal phase 2): PD feedback to balance the rod, closed loop

## 1. Where we start, what we want, what we get

| | |
|---|---|
| **We have** | the step 5 simulator, the 5 degree start, a target (upright, $q_{\text{des}}=0$, $\dot q_{\text{des}}=0$) |
| **Unknown** | the motion under feedback: does the rod reach upright and stay? |
| **Set to 0** | noise $=0$; the target angles and target rates |
| **NEW in this step** | the feedback torque from angle and rate errors |
| **What we get** | a balanced rod, a settling curve, and the closed-loop theory to compare with |
| **Pass rule** | hand vs MuJoCo with the same law: frictionless $10^{-6}$ rad, with friction $10^{-3}$ rad |

## 2. The equations

**7.1 PD law.** The command is computed from the current state before each step and held over the step:

$$c_{\text{cmd},i}=-k_p\,(q_i-q_{\text{des},i})-k_d\,(\dot q_i-\dot q_{\text{des},i})=-k_p\,q_i-k_d\,\dot q_i$$

The motor then clips and scales it (5.1). Chosen gains: $k_p=9.81$ N·m/rad, $k_d=2.52$ N·m·s/rad.

**7.2 Closed loop near upright.** Small angles: $M\approx I_p$, $\mathbf h\approx0$, $\mathbf g\approx-U_0\,q$. Substituting 7.1 into the equation of motion:

$$I_p\,\ddot q_i+(k_d+b)\,\dot q_i+(k_p-U_0)\,q_i=0$$

**7.3 Stability condition and numbers.** This is a damped oscillator, so it is stable when both coefficients are positive:

$$k_p>U_0=4.905\ \text{N}\!\cdot\!\text{m/rad},\qquad k_d+b>0$$

$$\omega_n=\sqrt{\frac{k_p-U_0}{I_p}},\qquad \zeta=\frac{k_d+b}{2\sqrt{I_p\,(k_p-U_0)}}$$

With $k_p=9.81$: $\omega_n=3.836$ 1/s. With $k_d=2.52$ and $b=0$: $\zeta=0.985$, nearly critically damped.

**7.4 Initial torque and saturation.** At the start the command is $-k_p\,q_0=(+0.606,\ -0.605)$ N·m, well inside $\pm3$. The command saturates only when $k_p|q|>3$, that is $|q|>0.31$ rad.

**7.5 Friction offset.** With Coulomb friction the rod can stop short of upright while the net restoring torque is below $c$:

$$(k_p-U_0)\,|q_i|\le c\ \Rightarrow\ |q_i|\le\frac{0.02}{4.905}=0.004\ \text{rad}$$

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $k_p$ | proportional gain | N·m/rad | **NEW**, known input (9.81) | `proportional_gain` |
| $k_d$ | derivative gain | N·m·s/rad | **NEW**, known input (2.52) | `derivative_gain` |
| $q_{\text{des}},\ \dot q_{\text{des}}$ | targets | rad, rad/s | **set to 0** (upright, still) | `target_angles`, `target_rates` |
| $\omega_n,\ \zeta$ | closed-loop frequency, damping ratio | 1/s, 1 | output | `natural_frequency`, `damping_ratio` |

## 3. Plug in, get out

```
state (q, q_dot) at the start of the step
   -> 7.1 command -> 5.1 clip and gear -> u (held over the step)
   -> 5.3 q_ddot -> RK4 -> next state
   -> compare with MuJoCo using the same law
   -> compare decay with 7.3 (omega_n, zeta)
```

## 4. Conclusion

You hold a balanced rod. The closed loop turns the unstable growth rate of $+3.8$ 1/s into a stable decay with the same speed, set by $k_p$ and $k_d$.


---

# Step 8 (gimbal phase 2, optional): Computed torque tracking, closed loop

## 1. Where we start, what we want, what we get

| | |
|---|---|
| **We have** | the full model (the hand $M$, $\mathbf h$, $\mathbf g$, friction), a smooth reference trajectory $q_{\text{des}}(t)$ |
| **Unknown** | the torque that makes the rod follow the reference |
| **Set to 0** | noise $=0$ |
| **NEW in this step** | the computed torque law: the controller uses the model to cancel the physics |
| **What we get** | tracking whose error obeys a simple linear equation |
| **Pass rule** | hand vs MuJoCo plant: frictionless $10^{-6}$ rad, with friction $10^{-3}$ rad; error follows the analytic curve |

## 2. The equations

**8.1 Reference trajectory** (small circle, starts at upright):

$$q_{\text{des}}(t)=A\begin{pmatrix}\sin\omega_r t\\1-\cos\omega_r t\end{pmatrix},\quad \dot q_{\text{des}}=A\omega_r\begin{pmatrix}\cos\omega_r t\\\sin\omega_r t\end{pmatrix},\quad \ddot q_{\text{des}}=A\omega_r^{2}\begin{pmatrix}-\sin\omega_r t\\\cos\omega_r t\end{pmatrix}$$

$A=0.1$ rad, $\omega_r=\pi$ rad/s.

**8.2 Tracking error and desired error dynamics**

$$e=q-q_{\text{des}},\qquad \ddot e+k_{d}^{\text{ct}}\,\dot e+k_{p}^{\text{ct}}\,e=0$$

**8.3 Computed torque law.** Choose the accelerations we want, then ask the model for the torque that produces them:

$$\mathbf v=\ddot q_{\text{des}}-k_d^{\text{ct}}\,\dot e-k_p^{\text{ct}}\,e$$

$$\mathbf u=M(q)\,\mathbf v+\mathbf h(q,\dot q)+\mathbf g(q)+b\,\dot q+c\tanh\!\left(\frac{\dot q}{\varepsilon}\right)$$

**8.4 Why it works.** Substitute 8.3 into the equation of motion $M\ddot q+\mathbf h+\mathbf g=\mathbf u-b\dot q-c\tanh(\dot q/\varepsilon)$. The terms $\mathbf h,\ \mathbf g$, damping and friction cancel, leaving

$$M\ddot q=M\mathbf v\ \Rightarrow\ \ddot q=\mathbf v\ \Rightarrow\ \ddot e+k_d^{\text{ct}}\dot e+k_p^{\text{ct}}e=0$$

This holds only if the model is exact and the torque does not saturate.

**8.5 Analytic error for the critically damped choice.** With $k_p^{\text{ct}}=\omega_c^2$ and $k_d^{\text{ct}}=2\omega_c$ ($\omega_c=6$ 1/s), each component of $e$ is

$$e(t)=\big(e_0+(\dot e_0+\omega_c e_0)\,t\big)\,e^{-\omega_c t}$$

Our simulation holds the torque constant over each step, so it follows this curve up to a small discretization error that the notebook reports.

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $q_{\text{des}},\dot q_{\text{des}},\ddot q_{\text{des}}$ | reference and its derivatives | rad, rad/s, rad/s² | **NEW**, known input | `reference_angles`, `reference_rates`, `reference_accelerations` |
| $A,\ \omega_r$ | reference amplitude, frequency | rad, rad/s | known input (0.1, $\pi$) | `reference_amplitude`, `reference_frequency` |
| $k_p^{\text{ct}},\ k_d^{\text{ct}}$ | error gains | 1/s², 1/s | **NEW**, known input (36, 12) | `error_gain_p`, `error_gain_d` |
| $\mathbf v$ | commanded acceleration | rad/s² | **NEW**, output | `desired_acceleration` |
| $\mathbf u$ | computed torque | N·m | **NEW**, output | `motor_command` |

## 3. Plug in, get out

```
state (q, q_dot) + time t
   -> 8.1 reference -> 8.2 error -> 8.3 v -> M, h, g, friction terms -> command u
   -> 5.1 clip -> plant (hand model, or MuJoCo) -> next state
```

For the MuJoCo plant, the controller still uses the hand model functions, so agreement also tests the hand $M$, $\mathbf h$, $\mathbf g$.

## 4. Conclusion

You hold a controller that uses the model itself. If tracking is good, the model is good; this is the strongest check of phase 2.


---

# Step 9 (gimbal phase 2): Measurement noise

## 1. Where we start, what we want, what we get

| | |
|---|---|
| **We have** | a clean logged angle trajectory from the hand simulator (the PD balance run of step 7, with damping and friction on) |
| **Unknown** | none, we create the data a real sensor would give |
| **Set to 0** | nothing in the dynamics. The simulation stays noise-free, noise is added to the **logged** angles afterwards |
| **NEW in this step** | the noise model: Gaussian, optional quantization |
| **What we get** | noisy angle data with known $\sigma$, saved for Chapter 2 |
| **Check** | not against MuJoCo. The noise itself must have mean 0 and std $\sigma$ within sampling error |

## 2. The equations

**9.1 Gaussian measurement noise**

$$\tilde q_{k,i}=q_{k,i}+\eta_{k,i},\qquad \eta_{k,i}\sim\mathcal N(0,\sigma^2)\ \text{independent for each sample and each angle}$$

$\sigma=0.01$ rad (the value used in the earlier notebook). A fixed random seed makes the run repeatable.

**9.2 What to check.** With $N$ samples of $\eta$, the sample mean and sample std scatter around $0$ and $\sigma$ by

$$\text{std error of the mean}=\frac{\sigma}{\sqrt N},\qquad \text{std error of the sample std}\approx\frac{\sigma}{\sqrt{2N}}$$

We pass if both are within 4 standard errors.

**9.3 Encoder quantization (optional second noise).** A 12-bit encoder reads angles in steps of

$$\Delta_{\text{enc}}=\frac{2\pi}{4096}=1.534\times10^{-3}\ \text{rad},\qquad \tilde q=\Delta_{\text{enc}}\operatorname{round}\!\left(\frac{q}{\Delta_{\text{enc}}}\right)$$

The error is spread evenly over $\pm\Delta_{\text{enc}}/2$, with std $\Delta_{\text{enc}}/\sqrt{12}=4.4\times10^{-4}$ rad. Much smaller than $\sigma$.

**9.4 Why we do not difference noisy angles to get velocity**

$$\hat{\dot q}_k=\frac{\tilde q_{k+1}-\tilde q_k}{\Delta t}\ \Rightarrow\ \text{std}=\frac{\sqrt2\,\sigma}{\Delta t}=14.1\ \text{rad/s}$$

The real rates are about 1 rad/s, so the noise is 14 times larger than the signal.

**9.5 Link to Chapter 2.** The covariance of fitted parameters is approximately

$$\Sigma\approx\sigma^{2}\left(J^{\mathsf T}J\right)^{-1}$$

and $\sigma$ here is exactly the $\sigma$ of 9.1.

| Symbol | Meaning | Unit | Status | Code name |
|---|---|---|---|---|
| $\sigma$ | noise std | rad | **NEW**, known input (0.01) | `noise_std` |
| $\eta$ | noise samples | rad | **NEW**, output | `noise_samples` |
| $\tilde q$ | noisy measurement | rad | **NEW**, output | `noisy_angles` |
| $\Delta_{\text{enc}}$ | encoder step | rad | **NEW**, known input | `encoder_step` |
| $N$ | number of samples | 1 | output | `n_samples` |

## 3. Plug in, get out

```
clean angles q (from the PD run) + sigma + seed
   -> 9.1 draw eta, add -> noisy angles
   -> 9.2 check mean and std of (noisy - clean)
   -> 9.3 quantized version, 9.4 differenced velocity noise
   -> save clean and noisy arrays for Chapter 2
```

## 4. Conclusion

You hold data that looks like a measurement: clean physics plus known-size noise. The overlay in step 4 still compares clean with clean. The noisy arrays go to Chapter 2.
