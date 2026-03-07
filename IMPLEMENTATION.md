# GPS + IMU Sensor Fusion

## The Problem

GPS gives us position but updates slowly and is noisy. The IMU gives us acceleration at high frequency but drifts over time. The goal is to combine both to get a smoother, more accurate position and velocity estimate.

---

## Step 1 — Convert GPS to Metres

The UKF works in metric space. GPS coordinates (degrees) are converted to a **signed distance in metres** from a fixed local origin using the Haversine formula:

$$a = \sin^2\!\left(\frac{\Delta\phi}{2}\right) + \cos\phi_1 \cdot \cos\phi_2 \cdot \sin^2\!\left(\frac{\Delta\lambda}{2}\right)$$

$$d = 2R_E \cdot \text{atan2}\!\left(\sqrt{a},\ \sqrt{1-a}\right)$$

The problem is split into two independent axes — **North** (latitude) and **East** (longitude) — each handled by its own filter.

---

## Step 2 — The State Model

Each filter tracks position and velocity along one axis:

$$\mathbf{x}_k = \begin{bmatrix} p_k \\ v_k \end{bmatrix}$$

The IMU acceleration $a_k$ drives the prediction at each step:

$$p_k = p_{k-1} + v_{k-1}\,\Delta t + \tfrac{1}{2}\,a_k\,\Delta t^2$$
$$v_k = v_{k-1} + a_k\,\Delta t$$

GPS measures position and velocity directly, so the measurement model is simply:

$$\mathbf{z}_k = \begin{bmatrix} p_k \\ v_k \end{bmatrix}$$

---

## Step 3 — The Unscented Kalman Filter

A standard Kalman Filter linearises nonlinear functions. The UKF avoids this by using the **Unscented Transform** — it generates $2n+1$ representative points (sigma points) around the current estimate, propagates each one through the model, and recombines them to recover the new mean and covariance.

With state dimension $n = 2$, this gives **5 sigma points** per step.

### Predict

$$\hat{\mathbf{x}}_{k|k-1},\ \mathbf{P}_{k|k-1} = \text{UnscentedTransform}(f,\ \mathbf{x}_{k-1},\ \mathbf{P}_{k-1}) + \mathbf{Q}$$

### Update (when GPS is available)

$$\mathbf{K}_k = \mathbf{P}_{xz}\,\mathbf{S}_k^{-1}$$

$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k\left(\mathbf{z}_k - \hat{\mathbf{z}}_k\right)$$

$$\mathbf{P}_k = \mathbf{P}_{k|k-1} - \mathbf{K}_k\,\mathbf{S}_k\,\mathbf{K}_k^\top$$

The Kalman gain $\mathbf{K}_k$ balances trust between the model prediction and the GPS measurement based on their relative uncertainties.

---

## Step 4 — Noise Matrices

**Process noise Q** — how IMU uncertainty grows over time:

$$\mathbf{Q} = \sigma_a^2 \begin{bmatrix} \dfrac{\Delta t^4}{4} & \dfrac{\Delta t^3}{2} \\[8pt] \dfrac{\Delta t^3}{2} & \Delta t^2 \end{bmatrix}$$

**Measurement noise R** — GPS position and velocity are independent with different accuracies:

$$\mathbf{R} = \begin{bmatrix} \sigma_p^2 & 0 \\ 0 & \sigma_v^2 \end{bmatrix} = \begin{bmatrix} 6.25\ \text{m}^2 & 0 \\ 0 & 0.01\ \text{(m/s)}^2 \end{bmatrix}$$

**Initial covariance** — seeded from the GPS position accuracy at startup:

$$\mathbf{P}_0 = \sigma_p\,\mathbf{I} = \begin{bmatrix} 2.5 & 0 \\ 0 & 2.5 \end{bmatrix}$$

---

## Step 5 — Back to Coordinates

After filtering, the metric estimates $(p_N, p_E)$ are converted back to geographic coordinates using the forward azimuth formula — first applying $p_N$ northward, then $p_E$ eastward from the local origin.
