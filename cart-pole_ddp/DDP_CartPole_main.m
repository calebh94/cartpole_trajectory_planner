%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% Iterative Linear Quadratic Regulator for Cart-Pole Dynamics                 %%%%%%%%%%%%%%%%%
%%%%%%%%%%%                  Discrete-Time Implementation                               %%%%%%%%%%%%%%%%%
%%%%%%%%%%% Based on Maxim Goldshtein's DDP code for AE8803THE                          %%%%%%%%%%%%%%%%% 
%%%%%%%%%%% Re-written and heavily modified on 6/3/2016 by Panagiotis Tsiotras          %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;  close all; format compact

global E

% --- cart-pole parameters
E.mp     = 0.5;   % mass of pole
E.mc     = 1;     % mass of cart
E.muc    = 0.05;  % cart friction
E.mup    = 0.05;  % pole friction
E.l      = 1;     % pole length
E.g      = 9.8;   % gravity

% Final Time
Tf = 10;

% Discretization
dt = 0.1;

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

%------------------------------------------------------------------
%                  Collect Data
%------------------------------------------------------------------

datain.gamma = gamma;
datain.auxdata.target = target;
datain.xo = x0;
datain.u_k = u_k;
datain.num_iter = num_iter;
datain.t_k = t_k;
datain.Horizon = horizon;
datain.dt = dt;
datain.Tf = Tf;
datain.reg_con = reg_con;
datain.EOMfile = @EOM_CartPole;
datain.COSTfile = @(x_,u_,t_,target) Cost_CartPole(x_,u_,t_,target);

%------------------------------------------------------------------
%                  Call Discrete DDP 
%------------------------------------------------------------------
tic;
sol = DDP_discrete(datain);
toc;

%------------------------------------------------------------------
%                   Plot resuts
%------------------------------------------------------------------

PlotResults(sol);


