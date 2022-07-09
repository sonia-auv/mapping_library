classdef Preprocessing  < handle
    %POINTCLOUDFILTER classe mere
    
    properties (Access = private)
        % ROS params
        param;
    end

    methods
        %% Preprocessing Constructor
        function this = Preprocessing(param)   
            % ROS Subscribers
            this.param = param;
        end

        function out = filter(this, xyzi)
            rowsToDelete = any( xyzi(:,4) < this.param.minIntensity ...
                              | xyzi(:,4) > this.param.maxIntensity ...
                              | sqrt(xyzi(:,1).^2+xyzi(:,2).^2+xyzi(:,3).^2) < this.param.minRange ...
                              | sqrt(xyzi(:,1).^2+xyzi(:,2).^2+xyzi(:,3).^2) > this.param.maxRange , 2);
            xyzi(rowsToDelete, :) = [];
            out = xyzi;
            return;
        end

        function setParam(this, param)
            this.param = param;
        end
    end
end

