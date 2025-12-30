The point is to follow a reference signal while preventing aggressive control. Maximum change of control per timestep is restricted to be max_dd = 0.07, while control signal d is between [0,1].

OptimalDCMotorControl.mlx = run all scripts below in order

dcm_sys.m = define system model

dcm_lqr.m = simulate linear quadratic regulator

dcm_ocp.m = solve a reference tracking problem (preliminary for MPC)

dcm_mpc.m = simulate model predictive control

Note that Casadi is needed to run the OCP solver.

This project was carried out on the "Numerical Optimal Control in Science and Engineering" course at Albert-Ludwigs-Universität in Freiburg, Germany.
