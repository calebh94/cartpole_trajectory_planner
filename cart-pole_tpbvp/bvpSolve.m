%%
% Define function to solve problem
function answer = bvpSolve(a,b,c,d,e,f,g)
global A B x0 Q R S
global tf dt

% tf=10; x0 = [5, 2]';
Q = a; R = b; S = c;
A = d; B = e; x0 = f; tf = g;
% tf = 10; dt = 0.1; 
time = [0:dt:tf];
solinit = bvpinit(linspace(0,tf), @bvpInit);
sol = bvp4c(@bvpODE, @bvpBC, solinit);
time = sol.x;
state = sol.y([1,2],:);
lambda = sol.y([3,4],:);
u=-inv(R)*B'*sol.y([3,4],:);
answer(1,:) = time; answer([2 3],:) = state; 
answer([4 5],:) = lambda; answer(6,:) = u;
end

% Define ode function
function H = bvpODE(t,y)
global A B x0 Q R S
% ODE Function for the Hamiltonian Matrix for Two-Point Boundary Layer
% [xdot, lambadot] = H [x, lambda)
H = [A, -B/(R)*B';
     -Q, -A';]*y;
end

%%

function BC = bvpBC(ya,yb)
global A B x0 Q R S
BC = [ya(1) - x0(1); ya(2) - x0(2); yb(3:4) - S*yb(1:2)];
end
%%

function in = bvpInit(t)
global A B x0 b alp
in = [x0; 1; 0];
end


