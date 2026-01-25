% *** This script implements MPC with a Kalman filter ***
% Significant noise is added to demonstrate how feedback control
% and Kalman filtering enhances performance

% Path vector, objective function weights and max_dd are defined 
% in dcm_ocp.m

control_window = 5;    % control horizon lenght
N_mpc = 2;             % Horizon length (s)
t_mpc = 0:Ts:N_mpc;    % Horizon timepoints 
% Add extra points to ref path to ensure succesful MPC loop
ref_mpc = [ref, ones(1,control_window)*ref(end)];
% Decision variables
D = casadi.MX.sym('D',1,control_window); % controls: duty cycles

% Parameters
% Shaft speed reference
ref_path_param = casadi.MX.sym('ref_path',1,control_window);
% Initial state parameter
x0_param = casadi.MX.sym('x0',2,1);  
% d0_param is needed because max change of D in one timestep 
% is constrained
d0_param = casadi.MX.sym('d0',1,1);


% Initilialize state vectors
x_mpc = casadi.MX.zeros(2,control_window);
x_mpc(:,1) = x0_param;                  
for i=1:control_window-1
    x_mpc(:,i+1) = Ad*x_mpc(:,i) + Bd*Vi*D(i);   
end

% weigths q and r are definde in dcm_ocp.m

% Calculate objective
J = 0.0 ; 
for i=1:control_window
    % Reference speed path error cost
    path_error = ref_path_param(i) - x_mpc(1,i);    
    J = J + transpose(path_error) * q * path_error ;  
    % Control cost
    J = J + transpose(D(i)) * r * D(i) ;
end


% Define constraints
% max_dd is defined in dcm_ocp (controls max change per timestep)
g = [D(1)-d0_param]; 
lbg = [-max_dd];                
ubg = [max_dd];
for k=1:control_window
    % 0 <= d <= 1
    g = [g; D(k)];
    lbg = [lbg; 0];
    ubg = [ubg; 1];
    % -max_dd <= d(k+1) - d(k) <= max_dd
    if(k>1)
        g = [g; D(k) - D(k-1)];
        lbg = [lbg; -max_dd];
        ubg = [ubg; max_dd];
    end

end


% MPC loop initializations
x = [0;0];       % Initial state based on system model
d = 0;           % Initial control
% Initialize solution matrices
dSol_mpc = zeros(1,length(t_mpc));
xSol_mpc = zeros(2,length(t_mpc));
xSol_mpc(:,1) = x_init; 
% Initialize ocp struct
ocp = struct();  
ocp.x = [D] ;
ocp.p = [reshape(x0_param,[1,2]), d0_param, ref_path_param] ;
ocp.g = transpose(g) ;
ocp.f = J ;
% Initialize solver
solver = casadi.qpsol('solver','qpoases',ocp);
% Kalman filter initial values
Pk = 10*eye(2);          % State covariance
Qk = diag([0.1, 0.1]);   % Process noise covariance
Rk = 10;                 % Measurement noise covariance

% Moving horizon loop
tic;
for k=1:length(t_mpc)-1
    % Update window 
    window = ref_mpc(k:k+control_window-1);
    % Solve the optimal control problem for current time frame
    solution = solver('p',[transpose(x),d,window], 'lbg',lbg, 'ubg',ubg);
    d = full(solution.x(1)); % New optimal control

    %{ Apply Kalman filter
    xp = (Ad*x + Bd*Vi*d);       % Prediction based on system model
    w = randi([-1000,1000])*1e-3*ones(2,1); % Process noise
    v = randi([-1000,1000])*1e-3;           % Measurement noise
    xr = xp + w ;                     % Real system state
    y = Cd*xr + v ;                   % Real measurement
    Pk = Ad*Pk*Ad' + Qk;              % Update Pk  
    Kk = Pk*Cd'/(Cd*Pk*Cd' + Rk);     % Kalman gain
    x = xp + Kk*(y-Cd*xp);            % State estimate
    Pk = Pk - Kk*Cd*Pk;               % Update Pk

    % Add d to dSol and x to xSol for plotting 
    dSol_mpc(k) = d;
    xSol_mpc(:,k+1) = x;

end
toc;

% Plot reference signal
figure;
plot(t_mpc,ref)
title('MPC Shaft Speed Reference')
xlabel('t')
ylabel('\omega_ref')
ylim([-1,18]);
xlim([0,2.1]);

% Plot controls
figure;
plot(t_mpc(1:end),dSol_mpc)
xlabel('t')
ylabel('D')
ylim([-0.1,1.1]);
xlim([0,2.1]);
title('MPC Controls')

% Plot reference path and actual path
figure;
hold on;
title('MPC Ref. and Real Path (with noise) + Kalman filter')
ylabel('\omega')
xlabel('t')
plot(t_mpc,ref,'red');
plot(t_mpc, xSol_mpc(1,:),'green');
ylim([-1,18]);
xlim([0,2.1]);
legend('Ref','Real');
hold off;


% Check control change rate and verify that max_dd is met
ddSol_mpc = diff(dSol_mpc);
figure;
plot(t_mpc(1:end-1),ddSol_mpc);
title('MPC Control Rate of Change')
xlabel('t')
ylabel('dd/dt')
disp('Controls, Max Rate of Change:')
disp(max(ddSol_mpc))
if abs(max(ddSol_mpc) - max_dd) < 5e-16
    disp('max_dd is met')
else
    disp('max_dd is not met')
end

