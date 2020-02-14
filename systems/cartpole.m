classdef cartpole
    %CARTPOLE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        mp
        mc
        muc
        mup
        l
        x
        xdot
        theta
        thetadot
        u
    end
    
    methods
        function obj = cartpole(mp,mc,muc,mup,l)
            %CARTPOLE Construct an instance of this class
            %   Detailed explanation goes here
            obj.mp = mp;
            obj.mc = mc;
            obj.muc = muc;
            obj.mup = mup;
            obj.l = l;
        end
        
        function obj = initState(obj,x0,xdot0,theta0,thetadot0)
            obj.x = x0;
            obj.xdot = xdot0;
            obj.theta = theta0;
            obj.thetadot = thetadot0;
            obj.u = 0;  % init control as 0
        end
        
        function cost = calculateCost(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            cost = 1000;
        end
        
        function obj = dynamics(obj, u, dt)
            xt = [obj.x, obj.xdot, obj.theta, obj.thetadot]';
            [dXdt, xdot_x, xdot_u] = EOM_CartPole(xt,u, obj);
            xtnew = dXdt * dt + xt;
            obj.x = xtnew(1);
            obj.xdot = xtnew(2);
            obj.theta = xtnew(3);
            obj.thetadot = xtnew(4);
        end            
    end
end

