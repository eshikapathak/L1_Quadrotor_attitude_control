# L1 Adaptive Control & MRAC for Quadrotor Attitude Control

This repository contains a simulation environment for comparing **L1 Adaptive Control, Model Reference Adaptive Control (MRAC)**, and **Baseline LQR** for quadrotor attitude tracking.

The project is built on MATLAB/Simulink and includes a master script to automate various experiments, including disturbance rejection, reference tracking, and parameter sweeps (sampling time, filter bandwidth, delay margin).

## 📂 File Structure

`master_experiments.m`: The master MATLAB script. This is the only file you need to interact with. It initializes parameters ($P$), configures the simulation, runs specific experiments based on user input, and generates analysis plots.

`l1_lqr.slx`: The Simulink model containing the plant dynamics, LQR baseline, L1 architecture (Predictor, Adaptation Law, Filter), and MRAC architecture.

## 🚀 How to Run

1. Open MATLAB.

2. Ensure both `master_experiments.m` and `l1_lqr.slx` are in the current working directory.

3. Type master_experiments in the Command Window and press Enter.

4. The script will pause and display a menu of sample Experiments.

5. Enter the number of the experiment you wish to run (e.g., 1 for basic Disturbance Rejection) and press Enter.

## 🧪 Available Experiments

The script supports the following test scenarios:

   * Disturbance Rejection: Compare LQR vs L1+LQR vs MRAC holding a 30-degree setpoint against sinusoidal disturbance.

   * Tracking Performance: Compare controllers tracking Step ($10^\circ, 40^\circ$) and Sine wave references.

   * Ts Sweep: Analyze how Sampling Time ($T_s$) affects RMSE.

   * MRAC Gain Sweep: Plot RMSE vs Adaptation Gain ($\Gamma$).

   * Filter BW Sweep (Dist Freq): RMSE vs Filter Bandwidth ($w_c$) for different disturbance frequencies (1st Order LPF).

   * Filter BW Sweep (Dist Amp): RMSE vs Filter Bandwidth for different disturbance amplitudes.

   * Control Signal Analysis: Detailed time-domain and frequency-domain (FFT) plots of control signals for varying filter bandwidths.

   * Disturbance (Triangle): Tests rejection of non-smooth triangle wave disturbances.

   * State-Dependent Disturbance: Tests rejection of nonlinear $d(x) = 2\sin(3\phi)\dot{\phi}$ disturbance.

   * MRAC TDM vs Gain: Plots Time Delay Margin vs MRAC Adaptation Gain.

## ⚙️ Configuration & Tuning

All critical parameters are defined in the Global Initialization section at the top of master_experiments.m. You can tweak defaults here:

1. Plant & Physics

   * `P.m`, `P.Ix`, `P.Iy`, `P.Iz`: Mass and Inertia properties.

   * `P.L`: Quadrotor arm length.

2. Baseline LQR

   * `Q`, `R`: Weight matrices for the LQR design. Modify these to change the baseline stiffness.

3. L1 Adaptive Control
   
   * `P.Ts`: Sampling time for the piecewise constant adaptation (default 0.002s).
   
   * `P.wc`: Low-pass filter cutoff frequency (default 40 rad/s).
   
   * `P.Ae`: Error dynamics matrix (default -10 * eye(6)).

4. MRAC
    
    * `P.MRAC.Gam_x, Gam_r, Gam_w`: Adaptation rates ($\Gamma$).
    
    * `P.MRAC.theta_max`: Projection operator bounds to prevent parameter drift.

    * `Q_lyap`: Lyapunov equation parameter for adaptation law derivation.

5. Simulink Internals 
   If you need to modify the internal logic, double-click the MATLAB Function blocks in l1_lqr.slx:

     * `DisturbanceGen`: Logic for Sine (0), Sum of Sines (1), Triangle (2), and State-Dependent (3) disturbances.

     * `ReferenceGen`: Logic for Step (0) vs Sine (1) reference trajectories.

     * `StatePredictor`: The linear/nonlinear(commented-out) predictor dynamics 

     * `MRAC_Controller`: Implementation of the MRAC adaptive laws with Projection Operator.

## ⚠️ Troubleshooting & Notes

* Transport Delay Warnings: If you see warnings about "delay smaller than step size," this is expected during Time Delay Margin (TDM) sweeps where we test delays close to zero. The script handles this by forcing a small non-zero delay (1e-7) for the zero case.
* Slow Simulations (TDM Sweeps): Experiments may run slower because they sweep the system to the point of instability. For these experiments, you can use specific solver settings (ode23t with relaxed tolerances) to keep runtimes reasonable.
* Algebraic Loops: The model uses a Transport Delay (or Memory block) in the feedback loop to break algebraic loops required by the L1 architecture. Do not remove this delay block.
* Control affine state-predictor: We have also provided code (commented out in the state-predictor block) for a state-predictor of the form $f(x, u) + Bu$.
* Aggressive LQR: Making the LQR params larger/aggressive can lead to a slower simulation, i.e. the experiment takes longer to end. 
