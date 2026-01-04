% This script implements an OCP and solves it
% Add significant noise to demonstrate how open loop control
% can perform poorly

N_opc = 2;               % Time horizon
t_opc = 0:Ts:N_opc;      % Time points

% Reference signal for one horizon
% Uncomment the one to be used
 
% *** REF #1: simple step signal ***
%ref = [zeros(1,50), ones(1,151)*7]; 

% *** REF #2: multiple steps ***
%ref = [ones(1,10),ones(1,90)*12,ones(1,50)*5,ones(1,51)*14];

% *** REF #3: mixed step and sinusoid ***
%ref = [ones(1,50)*5,0.5*sin(0:49)+3,ones(1,51)*13,0.3*cos(0:49)+4];      

% *** REF #4: mixed step and linear growth/decay ***
ref = [ones(1,25)*3, ones(1,25)*6, 0.5*(0:14)+3,ones(1,14)*10, -0.5*(0:20)+10, zeros(1,25),ones(1,26)*2, 0.5*(0:24)+2,ones(1,25)*16];

% *** REF #5: mixed step, sinusoid and exponentials
%ref = [ones(1,25)*3, ones(1,25)*13, 0.2*sin(0:24)+0.3, exp(0:9)*0.001,exp(-(-9:0))*0.001, ones(1,25)*16, ones(1,25)*2, ones(1,15)*12,exp(0:9)*0.001, 0.15*sin(0:24)+0.45, exp(-(-9:-4))*0.001];


% Plot reference signal
figure;
plot(t_opc,ref)
title('OCP Shaft Speed Reference')
xlabel('t')
ylabel('\omega_ref')
ylim([-1,18]);
xlim([0,2.1]);

% *** Optimal Control Problem ***
% Decision variables
D = casadi.MX.sym('D',1,length(t_opc)-1);   % controls: duty cycle

% Parameters
ref_path_param = casadi.MX.sym('ref_path',1,length(t_opc));
x0_param = casadi.MX.sym('x0',2,1);

% Initilialize state vectors
x_opc = casadi.MX.zeros(2,length(t_opc));
x_opc(:,1) = x0_param;                  
for i=1:length(t_opc)-1
    x_opc(:,i+1) = Ad*x_opc(:,i) + Bd*Vi*D(i);   
end

% weigths
% Apply Bryson's rule 
q = [1000] ; % path error weight 
r = [100]  ; % control weight

% Calculate objective
J = 0.0 ; 
for i=1:length(t_opc)-1
    % Reference speed path error cost
    path_error = ref_path_param(i) - x_opc(1,i);    
    J = J + transpose(path_error) * q * path_error ;  
    % Control cost
    J = J + transpose(D(i)) * r * D(i);
end
% Terminal cost
path_error = ref_path_param(end) - x_opc(1,end);
J = J + transpose(path_error) *q* path_error;

% Define constraints
max_dd = 0.07;  % controls max change per timestep
g = [];
lbg = [];                
ubg = [];
for k=1:length(t_opc)-1
    % 0 <= d <= 1
    g = [g; D(k)];
    lbg = [lbg; 0];
    ubg = [ubg; 1];
    % Terminal state must be reached
    g = [g; x_opc(1,end)-ref_path_param(end)];
    lbg = [lbg;0];
    ubg = [ubg;0];
    % -max_dd <= d(k+1) - d(k) <= max_dd
    if(k>1)
        g = [g; D(k) - D(k-1)];
        lbg = [lbg; -max_dd];
        ubg = [ubg; max_dd];
    end

end


% Optimal Control Problem formulation
ocp = struct();
ocp.x = [D] ;
ocp.p = [reshape(x0_param,[1,2]), ref_path_param] ;
ocp.g = transpose(g) ;
ocp.f = J ;

% Solution with QP-solver
% No guess for states needed
% Initial value and reference path parameters are given
x_init = [0,0];
solver = casadi.qpsol('solver','qpoases',ocp);
solution = solver('p',[x_init,ref], 'lbg',lbg, 'ubg',ubg);



% Solution: vector of optimal duty cycles
dSol_opc = full(solution.x);

% Calculate states based on dSol and x_init 
xSol_opc = zeros(2,length(t_opc));
xSol_opc(:,1) = transpose(x_init);
for i=1:length(dSol_opc)
    w = randi([-1000,1000])*1e-3*ones(2,1); % Process noise
    xSol_opc(:,i+1) = Ad*xSol_opc(:,i) + Bd*Vi*dSol_opc(i) + w;
end

% Plot controls
figure;
plot(t_opc(1:end-1),dSol_opc)
title('OCP Controls')
xlabel('t')
ylabel('D')
ylim([-0.1,1.1]);
xlim([0,2.1]);

% Plot reference path and actual path
figure;
hold on;
title('OPC Ref. Path and Real Path (with noise)')
ylabel('\omega')
xlabel('t')
plot(t_opc,ref,'red');
plot(t_opc,xSol_opc(1,:),'green');
ylim([-1,18]);
xlim([0,2.1]);
legend('Ref.','Real');
hold off;


