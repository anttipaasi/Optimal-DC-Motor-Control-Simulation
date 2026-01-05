% This script implements LQR

% Weights. Apply Bryson's rule
Q = [10 0; 0 10];                % State weights
R = [100] ;                      % Control weight
[K_fb,~,~] = dlqr(Ad,Bd,Q,R) ;   % K_fb = optimal feedback control matrix
N_lqr = 0.3;                     % Time horizon
t_lqr = 0:Ts:N_lqr;              % Time points  

x0_lqr = [3 0 8 ; 2 0 2];                % Initial values
omegaGoal = [1, 10, 100];                % Goal motor shaft speeds to be simulated 
iGoal = b*omegaGoal/K;                   % Goal state motor currents 
xGoal = [omegaGoal; iGoal];              % Goal state matrix
uGoal = 1/Vi*(pinv(Bd)*(I-Ad)*xGoal);    % Goal state controls
objective = zeros(1,length(omegaGoal));  % Initialized objective sums

% Save trajectories here
xSol_lqr = zeros(2*length(omegaGoal), length(t_lqr));   % omega and i trajectories  
uSol_lqr = zeros(length(omegaGoal), length(t_lqr));     % controls

% Loop through each omega
for j=1:length(omegaGoal)    
    % Solution Loop
    x_lqr = x0_lqr(:,j);
    for k=1:length(t_lqr)
        xSol_lqr((2*j-1):(2*j), k) = x_lqr; 
        xError = x_lqr - xGoal(:,j);      
        u = uGoal(j) - K_fb*(xError);  
        u = min(max(0,u),1);   % Clip unfeasible controls
        uSol_lqr(j,k) = u;                   
        objective(j) = objective(j) + transpose(xError)*Q*xError + transpose(u)*R*u;
        x_lqr = (Ad*x_lqr + Bd*Vi*u) + randi([-1000,1000])*0.00005*ones(2,1);  % update x and add noise
    end
             
end

% Plot LQR performance for different initial values and goal speeds
figure; 
hold on;
title('LQR Performance for different x_0 and \omega_g (with noise)')
ylabel('\omega')
xlabel('t')
legendStrings = strings(length(omegaGoal),1);
for i=1:length(omegaGoal)
    plot(t_lqr,xSol_lqr(2*i-1,:))
    legendStrings(i) = sprintf('\\omega_g = %g', omegaGoal(i));
end
legend(legendStrings)


% Plot controls
figure;
hold on
title('LQR Controls')
ylabel('D')
xlabel('t')
legendStrings = strings(length(omegaGoal),1);
for i=1:length(omegaGoal)
    plot(t_lqr,uSol_lqr(i,:))
    legendStrings(i) = sprintf('\\omega_g = %g', omegaGoal(i));
end
legend(legendStrings)