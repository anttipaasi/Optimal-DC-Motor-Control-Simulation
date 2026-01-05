% This script defines system dynamics
clear variables
addpath("C:\Users\OMISTAJA\casadi matlab")
import casadi.*

% Parameters
J = 0.01;  
b = 0.1;
K = 0.7; 
zeta = 1;

% Equations
syms wn L R  
eq1 = wn^2 == 1/(J*L) ;
eq2 = wn^2 == (R*b+K^2)/(J*L) ;
eq3 = (L*b+J*R)/(J*L) == 2*zeta*wn ; 

% Solve for L, R and wn
[L,R,wn] = solve(eq1,eq2,eq3, L,R,wn) ;
L = double(L(1))
R = double(R(1))
wn = double(wn(1))

% Continuous state-space model
Vi = 24;    % Converter input voltage 
A = [-b/J K/J; -K/L -R/L] ;
B = [0; 1/L] ;
C = [1 0] ;
sys = ss(A,B,C,0);

% Plot continuous step response
figure;
step(sys)
title('Continuous Step Response')
ylabel('\omega')

% Discrete state-space model
Ts = 0.01;  % sample time
I = eye(size(A));
Ad = I+A*Ts;      
Bd = B*Ts;        
Cd = C ;
sysd = ss(Ad,Bd,Cd,0,Ts);

% Plot discrete step response
figure;
step(sysd)
title('Discrete Step Response')
ylabel('\omega')


% Simulate the discrete system for different controls
N = 5;      % Time horizon
t = 0:Ts:N; % All time points
D_steps = [zeros(1,100), 0.9*ones(1,100), 0.6*ones(1,100), zeros(1,100),0.97*ones(1,101)];
x = [0.0;0.0] ;             % Initial value 
xSol = zeros(length(t),2);  % Solution matrix
for k=1:length(t)
    xSol(k,:) = x;
    x = Ad*x + Bd*Vi*D_steps(k);
end

% Plot controls
figure;
plot(t,D_steps, 'blue')
title('Controls')
xlabel('t')
ylabel('D')

% Plot system response
figure;
plot(t,xSol(:,1),'green')
title('System Response')
xlabel('t')
ylabel('\omega')

disp('System Step Info:')
disp(stepinfo(sysd))





