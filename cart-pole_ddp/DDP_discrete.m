 function sol = DDP_discrete(datain);

num_iter = datain.num_iter;
Horizon = datain.Horizon;
dt = datain.dt;
u_k = datain.u_k;
target = datain.auxdata.target;
gamma = datain.gamma;
xo = datain.xo;
tspan = datain.t_k;
systemEOM = datain.EOMfile;
systemCOST = datain.COSTfile;
reg_con = datain.reg_con;

nx = size(xo,1);

for k = 1:num_iter

    x_traj = fnsimulate(systemEOM,xo,u_k,dt);
    cost(:,k) =  fnCostComputation(systemCOST,x_traj,u_k,dt,target);   
    fprintf('DDP Iteration %d,  Current Cost = %.4f \n',k,cost(1,k));
        
for  j = 1:Horizon-1
       
    % quadratize cost, adjust for dt
      [l0,lx,lxx,lu,luu,lux] = systemCOST(x_traj(:,j),u_k(:,j),j,target);  
      L(j) = l0*dt;
      Lx(:,j) = lx*dt;  
      Lxx(:,:,j) = lxx*dt; 
      Lu(:,j) = lu*dt;   
      Luu(:,:,j) = luu*dt;
      Lux(:,:,j) = lux*dt;
      
      % linearize dynamics, adjust for dt
      [~, dfx, dfu] = systemEOM(x_traj(:,j),u_k(:,j));
      A(:,:,j) = eye(nx,nx) + dfx*dt;
      B(:,:,j) = dfu*dt; 
          
end

[V,Vx,Vxx] = systemCOST(x_traj(:,end),u_k(:,end),[],target);
       
% single DDP pass backward V, Vx, Vxx and forward kinear dynamics
[du,l,K]  = DDP_iteration(A,B,L,Lx,Lxx,Lu,Luu,Lux,V,Vx,Vxx,reg_con);
   
% update the control
u_new = u_k + gamma*du;
u_k = u_new;

%---------------------------------------------> Simulation of the Nonlinear System
%x_traj = fnsimulate(systemEOM,xo,u_new,dt,tspan);

if (k>1) && (abs(cost(1,k)-cost(1,k-1)) < 1e-4)
    disp('Iteration stopped owing to small cost improvement') 
    break
end
 
% End of DDP Iteration

end

%  Prepare output 

sol.state = x_traj;
sol.time = tspan;
sol.control = [u_k u_k(end)];
sol.cost = cost;
sol.target = target;
sol.gain = K;
sol.l = l;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost =  fnCostComputation(systemCOST,x,u,dt,xf)

n = size(u,2)+1;
 
cost = 0;
 
 for k =1:n-1

     cost = cost +  dt*systemCOST(x(:,k),u(:,k),k,xf);
       
 end

  cost = cost + systemCOST(x(:,end),u(:,end),[],xf);
 
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function x = fnsimulate(systemEOM,x,u,dt)

n = size(u,2)+1;

for k = 1:n-1
    
    x(:,k+1) = x(:,k) + dt*systemEOM(x(:,k),u(:,k));
    
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function  [dU,l_k,L_k] = DDP_iteration(A, B, L, Lx, Lxx, Lu, Luu, Lux, T, Tx, Txx, reg_factor)
% Iteration of the DDP procedure
%
% Usage:[ dU ] = DDP_iteration(A, B, L, Lx, Lxx, Lu, Luu, Lux, T, Tx, Txx, reg_factor)
%
%   A, B - system matrices (third dimension is for time index)
%   L, Lx, Lxx, Lu, Luu, Lux - running cost and derivatives of *continuous* time system
%   T, Tx, Txx - terminal const and derivatives.
%   reg-factor - regularization factor (optional, default value is 1e-5).
%
% Changelog:
% v0 - base version, includes basic regularization


if isempty(reg_factor)
    reg_factor = 1e-5;
end

Horizon = size(A, 3)+1;

m = size(B,2);
n = size(A,1);

% Value function initialization

Vxx(:,:,Horizon)= Txx;
Vx(:,Horizon) = Tx(:);
V(Horizon) = T;

%------------------------------------------------> Backpropagation of the Value Function
for j = (Horizon-1):-1:1
    
    
    % compute feedforward control (l_k) and feedback gains matrix (L_k)
    Q_uu(:,:,j) = B(:,:,j)'*Vxx(:,:,j+1)*B(:,:,j) + Luu(:,:,j); % *********
    Q_u(:,j) = Vx(:,j+1)'*B(:,:,j) + Lu(:,j); % *************************
    Q_xu(:,:,j) = A(:,:,j)'*Vxx(:,:,j+1)*B(:,:,j)+Lux(:,:,j)'; % **********
    Q_ux(:,:,j) = Q_xu(:,:,j)'; % ***************************************
    Q_x(:,j) = Vx(:,j+1)'*A(:,:,j)+Lx(:,j)'; % *****************************
    Q_xx(:,:,j) = A(:,:,j)'*Vxx(:,:,j+1)*A(:,:,j)+Lxx(:,:,j); % *********
    
    l_k(:,j) = -inv(Q_uu(:,:,j))*Q_u(:,j); % COMPLETED
    L_k(:,:,j)= -inv(Q_uu(:,:,j))*Q_ux(:,:,j); % COMPLETED
    
    % compute value function and its first and second derivatives (V,Vx,Vxx)         
    V(j) = V(j+1) + l_k(:,j)'*Q_u(:,j)+(1/2)*l_k(:,j)'*Q_uu(:,:,j)*l_k(:,j); % COMPLETED
    Vx(:,j) = Q_x(:,j) + L_k(:,:,j)'*Q_u(:,j) + Q_xu(:,:,j)*l_k(:,j) + L_k(:,:,j)'*Q_uu(:,:,j)*l_k(:,j); % COMPLETED
    Vxx(:,:,j) = Q_xx(:,:,j) + L_k(:,:,j)'*Q_ux(:,:,j)+Q_xu(:,:,j)*L_k(:,:,j)+L_k(:,:,j)'*Q_uu(:,:,j)*L_k(:,:,j); % COMPLETED
             
end

% Forward propagation of the dynamics
dx = zeros(n,1);
dU = zeros(m,Horizon-1);
for i=1:(Horizon-1)
    du = l_k(:,i) + L_k(:,:,i)*dx;    
    dx = A(:,:,i)*dx + B(:,:,i)*du;
    dU(:,i) = du;
end

% check is iteration succeeded
if any(isnan(dU))
    error('Diverged!');
end

end

