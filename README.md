The point is to follow a reference signal while preventing aggressive control. Maximum change of control per timestep is restricted to be max_dd = 0.07, while control signal d is between [0,1]. Three different optimal control schemes are simulated: LQR, open-loop optimal control and MPC.

Instructions about the files below. Note that Control System Toolbox and CasADi are needed to run the scripts.


Optimal DC Motor Control with a Switch-Mode Power Supply - Simulation = project report 

MotorParameterDesign.mlx = Just shows how some motor parameters were calculated

OptimalDCMotorControl.mlx = run all scripts below in order

dcm_sys.m = define system model

dcm_lqr.m = simulate linear quadratic regulator

dcm_ocp.m = simulate open-loop optimal control (preliminary for MPC)

dcm_mpc.m = simulate model predictive control



This project was carried out on the "Numerical Optimal Control in Science and Engineering" course at Albert-Ludwigs-Universität in Freiburg, Germany.
