%%
clear all;close all;
set(0, 'DefaultAxesFontSize', 14, 'DefaultAxesFontWeight','demi')
set(0, 'DefaultTextFontSize', 14, 'DefaultTextFontWeight','demi')

% Setting up problem and calling function
% global A B
A = [0 1; 0 0]; B = [0, 1]'; Q = [1, 0; 0, 1]; R = 1; S = [2,0; 0, 1];
tf = 10; dt = 0.1; time = [0:dt:tf];
x0 = [5, 2]';
result = bvpSolve(Q,R,S,A,B,x0,tf); % Solving via bvp4c

%%
P=zeros(2,2,length(time));K=zeros(1,2,length(time));
V=S;
for kk=0:length(time)-1
P(:,:,length(time)-kk)=V;
K(:,:,length(time)-kk)=inv(R)*B'*V;
Pdot=-V*A-A'*V-Q+V*B*inv(R)*B'*V;
V=V-dt*Pdot;
end  % Solving via integrating P backwards in time 

%%
% Simulating state
x1=zeros(2,1,length(time));xcurr1=[5 2]';
u1 = zeros(1,1,length(time)); ucurr1=[0];
Vx = zeros(2,1,length(time));
for kk=1:length(time)-1
x1(:,:,kk)=xcurr1;
u1(:,:,kk)=ucurr1;
u1(:,:,kk)=-K(:,:,kk)*x1(:,:,kk);
xdot1=(A-B*K(:,:,kk))*x1(:,:,kk);
xcurr1=xcurr1+xdot1*dt;
Vx(:,:,kk) = P(:,:,kk)*x1(:,:,kk);
end
%%

figure(1);clf
plot(time,squeeze(x1(1,1,:)),time,squeeze(x1(2,1,:)),'--','LineWidth',2),
xlabel('Time (sec)');ylabel('States');title('State Trajectory')
hold on;plot(result(1,:),result([2],:),'s',result(1,:),result([3],:),'o');hold off
legend('Closed-Loop x_1','Closed-Loop x_2','Open-Loop x_1','Open-Loop x_2')
% print -dpng -r300 numreg2.png

% Control actions
cl_u1 = zeros(100,1);
cl_u1(:,1) = result(6,:)';
cl_u1(101,1) = 0;
figure(2);clf
plot(time,squeeze(u1(1,1,:)),'o'), hold on
plot(time,cl_u1(:,1), '--','LineWidth',2); hold off;
xlabel('Time (sec)');ylabel('Control u(t)');title('Control Histories')
legend('Open-Loop u(t)','Closed-Loop u(t)');

figure(3);clf

plot(time,squeeze(Vx(1,1,:)),time,squeeze(Vx(2,1,:)),'--','LineWidth',2),
xlabel('Time (sec)');ylabel('Co-states');title('Co-state Trajectory')
hold on;plot(result(1,:),result([4],:),'s',result(1,:),result([5],:),'o');hold off
legend('V_x1','V_x2','Co-state 1','Co-state 2')
% print -dpng -r300 numreg2.png
