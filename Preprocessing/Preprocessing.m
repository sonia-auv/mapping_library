classdef Preprocessing  < handle
    %POINTCLOUDFILTER classe mere
    
    properties (Access = private)
        % Subscribers
        minIntensitySub;
        maxIntensitySub;
        minRangeSub;
        maxRangeSub;

%         % State
%         minIntensityState;
%         maxIntensityState;
%         minRangeState;
%         maxRangeState;

        % Condition value
        minIntensity;
        maxIntensity;
        minRange;
        maxRange;
    end

    methods
        %% Preprocessing Constructor
        function this = Preprocessing()   

            % General preprocessing value
%             this.minIntensityState = true;
%             this.maxIntensityState = false;
%             this.minRangeState = true;
%             this.maxRangeState = false;

%             this.minIntensity = 0.07;
%             this.maxIntensity = 1;
%             this.minRange = 0.1;
%             this.maxRange = 100;
            
            % Subscribers

            this.minIntensitySub = rossubscriber('/proc_mapping/preprocessing/minIntensity', 'std_msgs/Float32', @this.minIntensityCallback, "DataFormat", "struct");
            this.maxIntensitySub = rossubscriber('/proc_mapping/preprocessing/maxIntensity', 'std_msgs/Float32', @this.maxIntensityCallback, "DataFormat", "struct");
            this.minRangeSub = rossubscriber('/proc_mapping/preprocessing/minRange', 'std_msgs/Float32', @this.minRangeCallback, "DataFormat", "struct");
            this.maxRangeSub = rossubscriber('/proc_mapping/preprocessing/maxRange', 'std_msgs/Float32', @this.maxRangeCallback, "DataFormat", "struct");
        end

        function out = filter(this, xyzi)
            rowsToDelete = any( xyzi(:,4) < Preprocessing.persistentDataStore('minIntensityValue') ...
                              | xyzi(:,4) > Preprocessing.persistentDataStore('maxIntensityValue') ...
                              | sqrt(xyzi(:,1).^2+xyzi(:,2).^2+xyzi(:,3).^2) < Preprocessing.persistentDataStore('minRangeValue') ...
                              | sqrt(xyzi(:,1).^2+xyzi(:,2).^2+xyzi(:,3).^2) > Preprocessing.persistentDataStore('maxRangeValue') , 2);
            xyzi(rowsToDelete, :) = [];
            out = xyzi;
            return;
        end

%         function actualizefilter(this)
%             this.minIntensity = Preprocessing.persistentDataStore('minIntensityValue');
%             this.maxIntensity = Preprocessing.persistentDataStore('maxIntensityValue');
%             this.minRange = Preprocessing.persistentDataStore('minRangeValue');
%             this.maxRange = Preprocessing.persistentDataStore('maxRangeValue');
% %             this.minIntensityState = Preprocessing.persistentDataStore('minIntensityState');
% %             this.maxIntensityState = Preprocessing.persistentDataStore('maxIntensityState');
% %             this.minRangeState = Preprocessing.persistentDataStore('minRangeState');
% %             this.maxRangeState = Preprocessing.persistentDataStore('maxRangeState');
%         end
    end

    %% ROS Callbacks
    methods(Static, Access = private)

        function minIntensityCallback(src,msg)
            Preprocessing.persistentDataStore('minIntensityValue', msg.Data);
%             Preprocessing.persistentDataStore('minIntensityState', true);
            fprintf('INFO : proc mapping : min Intensity filter changed \n');
        end
        function maxIntensityCallback(src,msg)
            Preprocessing.persistentDataStore('maxIntensityValue', msg.Data);
            fprintf('INFO : proc mapping : max Intensity filter changed \n');
        end
        function maxRangeCallback(src,msg)
            Preprocessing.persistentDataStore('maxRangeValue', msg.Data);
            fprintf('INFO : proc mapping : max Range filter changed \n');
        end
        function minRangeCallback(src,msg)
            Preprocessing.persistentDataStore('minRangeValue', msg.Data);
            fprintf('INFO : proc mapping : min Range filter changed \n');
        end

        function out1 = persistentDataStore(variable, value)
    
            persistent minIntensityValue maxIntensityValue minRangeValue maxRangeValue; %minIntensityState maxIntensityState minRangeState maxRangeState;
    
            % Initial variables
            if isempty(minIntensityValue)
                minIntensityValue = 0.07;
%                 minIntensityState = false;
            end
            if isempty(maxIntensityValue)
                maxIntensityValue = 1.0;
%                 maxIntensityState = false;
            end
            if isempty(minRangeValue)
                minRangeValue = 0.1;
%                 minRangeState = false;
            end
            if isempty(maxRangeValue)
                maxRangeValue = 100.0;
%                 maxRangeState = false;
            end
    
            if nargin == 1 % GET
                switch true
                    case (strcmpi(variable,'minIntensityValue') == 1)
                        out1 = minIntensityValue;
%                         out2 = minIntensityState;
                        return
                    case (strcmpi(variable,'maxIntensityValue') == 1)
                        out1 = maxIntensityValue; 
%                         out2 = maxIntensityState;
                        return
                    case (strcmpi(variable,'minRangeValue') == 1)
                        out1 = minRangeValue;
%                         out2 = minRangeState;
                        return
                    case (strcmpi(variable,'maxRangeValue') == 1)
                        out1 = maxRangeValue;
%                         out2 = maxRangeState;
                        return
    
                    otherwise
                        out1 = [];  
%                         out2 = [];
                end   
            elseif nargin == 2 % SET
                switch true
                    case (strcmpi(variable,'minIntensityValue') == 1)
                        minIntensityValue = double(value);
                        return
                    case (strcmpi(variable,'maxIntensityValue') == 1)
                        maxIntensityValue = double(value);
                        return
                    case (strcmpi(variable,'minRangeValue') == 1)
                        minRangeValue = double(value);
                        return
                    case (strcmpi(variable,'maxRangeValue') == 1)
                        maxRangeValue = double(value);
                        return
%                     case (strcmpi(variable,'minIntensityState') == 1)
%                         minIntensityState = value;
%                         return
%                     case (strcmpi(variable,'maxIntensityState') == 1)
%                         maxIntensityState = value;
%                         return
%                     case (strcmpi(variable,'minRangeState') == 1)
%                         minRangeState = value;
%                         return
%                     case (strcmpi(variable,'maxRangeState') == 1)
%                         maxRangeState = value;
%                         return
                end 
            end
        end
    end 
end

