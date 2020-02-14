classdef ddp
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Tf
        dt
        horizon
        tN
        num_iter
        targetx
        Qf
        Q
        R
        gamma
        reg_con
    end
    
    methods
        function obj = untitled3(inputArg1,inputArg2)
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

