function MovieFile = PlotResults(sol)

time = sol.time;
state = sol.state;
control = sol.control;
cost = sol.cost;
target = sol.target;
K = sol.gain;
l = sol.l;

% Plot Results

% figure('Position',[300 100 624 564]);
   

% Plot States: x [m], v [m/sec], theta [deg], thetadot [deg/sec]

% subplot(3,2,1);
figure(1);
plot(time,state(1,:),'g-',time,linspace(target(1),target(1),length(time)),'r--');
title('Position');
xlabel('time [s]'), ylabel('x [m]');
% subplot(3,2,2); 
figure(2);
plot(time,state(2,:));
plot(time,state(2,:),'g-',time,linspace(target(2),target(2),length(time)),'r--');
title('Velocity');
xlabel('time [s]'), ylabel('V [m/s]');
% subplot(3,2,3);
figure(3);
plot(time,state(3,:),'g-',time,linspace(target(3),target(3),length(time)),'r--');
title('Angular Position');
xlabel('time [s]'), ylabel('Angular Position [rad]');
% subplot(3,2,4);
figure(4);
plot(time,state(4,:),'g-',time,linspace(target(4),target(4),length(time)),'r--');
title('Angular Velocity');
xlabel('time [s]'), ylabel('Angular Velocity [rad/s]');

    
% Plot cost vs number of iterations

iterations = linspace(1,300,300);
figure(5);
plot(iterations, cost);
title('Cost vs. Iterations');
xlabel('Iterations'), ylabel('Cost');

% Plot controller gains
figure('Position',[600 100 524 564]);
plot(time(1:499), squeeze(K(1,1,:)),'g',time(1:499), squeeze(K(1,2,:)),'r',time(1:499), squeeze(K(1,3,:)), 'b',time(1:499),  squeeze(K(1,4,:)),'c');
title('Controller Gains for DDP Solution');
xlabel('Time (s)'), ylabel('Controller Gains');
legend('Gain 1', 'Gain 2', 'Gain 3', 'Gain 4');

%  animation
   
len = 1;

figure('Color','white','Position',[600 100 524 564]);
hh3 = plot([-1, 1], [0, 0], '.-','Markersize',5,'LineWidth',10);
hold on;
hh2 = plot([0, 0], [0, len*cos(state(3,1))], ...
    '.-', 'MarkerSize', 10, 'LineWidth', 5);
axis equal
axis([-1 5 -1.2 1.2])
ht = title(sprintf('Time: %0.2f sec', time(1)));

pos = get(gcf,'Position');
width = pos(3);
height = pos(4);

mov = zeros(height, width, 1, length(time), 'uint8');

for id = 1:length(time)
%     set(hh1(1), 'Xdata',time(id),'Ydata', state(3,id));
    set(hh3, 'XData',[(state(1,id)-0.5) (state(1,id)+0.5)], 'Ydata',[0 0]);
    set(hh2, 'XData', [state(1,id) (state(1,id)+len*sin(state(3,id)))],'Ydata',[0 (len*cos(state(3,id)))]);
    set(ht, 'String', sprintf('Time: %0.2f sec', time(id)))

    f = getframe(gcf);
    
    if id == 1
        [im(:,:,1,id),map] = rgb2ind(f.cdata,256,'nodither');
    else
        im(:,:,1,id) = rgb2ind(f.cdata,map,'nodither');
    end
%      mov(:,:,1,id) = rgb2ind(f.cdata,256,'nodither');

end

imwrite(im, map, 'animation.gif', 'DelayTime', 0, 'LoopCount', inf)

    
end







      
