% DDP Main

% Access to systems
addpath ../systems

clear all;  close all; format compact

% Select system (later)
system = cartpole(.5,1.0,0.05,0.05,1);

% load parameters from elsewhere (later)
% Final Time
Tf = 5;

% Discretization
dt = 0.01;

% Horizon 
horizon = Tf/dt;                  % 1.5sec
% Time span
t_k = linspace(0,Tf,horizon);
% Number of Iterations
num_iter = 300;
% Initial Control:
u0 = 0;
% Initial Configuration:
x0 = [0 0 pi 0.2]';
% Target 
target = [3 0 0 0]';
% Weight in Final State
E.Qf     = 10*diag([100 10 50 10]);
% Weight in control:
E.R  = .01;
% Weight in state:
E.Q  = diag([10 10 10 10]);
% Initial Control:
u_k = zeros(1,horizon-1);
% Learning Rate:
gamma = 0.1;
% regularization constant - default is 1e-5;
reg_con = 0.001;
