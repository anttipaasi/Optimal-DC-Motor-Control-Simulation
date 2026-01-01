% *** This script implements MPC ***

% path, objective function weights and max_dd are defined in dcm_ocp.m

% Formulate problem and solve for moving time frame
control_window = 5;   % control horizon lenght
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
for i=1:control_window-1
    % Reference speed path error cost
    path_error = ref_path_param(i) - x_mpc(1,i);    
    J = J + transpose(path_error) * q * path_error ;  
    % Control cost
    J = J + transpose(D(i)) * r * D(i) ;
end
% Terminal cost
path_error = ref_path_param(end) - x_mpc(1,end);
J = J + transpose(path_error) *q* path_error;

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

x = [0;0];       % Initial state
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
solver = casadi.qpsol('solver','qpoases',ocp);
% Moving horizon loop
tic;
for k=1:length(t_mpc)-1
    % Update window
    window = ref_mpc(k:k+control_window-1);
    % Solve the optimal control problem for current time frame
    solution = solver('p',[transpose(x),d,window], 'lbg',lbg, 'ubg',ubg);
    % Apply the first control => get new x and add noise
    d = full(solution.x(1));
    x = (Ad*x + Bd*Vi*d) + randi([-1000,1000])*0.00005*ones(2,1); 
    % Add to dSol and xSol for plotting
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


% Plot controls
figure;
plot(t_mpc(1:end),dSol_mpc)
xlabel('t')
ylabel('D')
title('MPC Controls')

% Plot reference path and actual path
figure;
hold on;
title('MPC Ref. and Real Path (with noise)')
plot(t_mpc,ref,'red');
plot(t_mpc, xSol_mpc(1,:),'green');
legend('Ref','Real');
hold off;


% Check control change rate and verify that max_dd is met
ddSol_mpc = diff(dSol_mpc);
figure;
plot(t_mpc(1:end-1),ddSol_mpc);
title('Control Rate of Change')
xlabel('t')
ylabel('dd/dt')
disp('Controls, Max Rate of Change:')
disp(max(ddSol_mpc))
if abs(max(ddSol_mpc) - max_dd) < 5e-16
    disp('max_dd is met')
else
    disp('max_dd is not met')
end