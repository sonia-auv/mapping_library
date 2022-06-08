classdef Mapper < handle
    % MAPPER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        % map3D; % Occupancy map.
        bigCloud;
        player;

        % Subscribers
        poseSub;
        startStopSub;
        sonarSub;
    end
    
    methods
        %% Mapper Constructor
        function this = Mapper()           
            % this.map3D = occupancyMap3D(10);
            this.bigCloud = pointCloud(zeros([1, 3]), 'Intensity', 0);
            this.player = pcplayer([-20 20],[-20 20],[0 5], 'VerticalAxisDir','Down');

            % Subscribers
            this.poseSub = rossubscriber('/proc_nav/auv_pose', 'geometry_msgs/Pose', "DataFormat", "struct");
            this.startStopSub = rossubscriber('/proc_mapping/start_stop', 'std_msgs/Bool', @this.startStopCallback, "DataFormat", "struct");
            this.sonarSub = rossubscriber('/provider_sonar/point_cloud2', 'sensor_msgs/PointCloud2', @this.sonarCallback, "DataFormat", "struct");
        end
        %% ROS Spin
        function spin(this, spin)
            killNode = false;
            reset(spin);
            fprintf('INFO : proc mapping : Node is started \n');
            fprintf('INFO : proc mapping : Wait for laser scan \n');
            
            while ~killNode
                % Add the laser scan to the point cloud when received.
                if this.persistentDataStore('newSonarMsg')
                    this.add2PtCloud(this.sonarSub.LatestMessage, this.poseSub.LatestMessage);
                    this.persistentDataStore('newSonarMsg', false);
                end
                waitfor(spin);
            end
        end

        %% Adding to the point cloud.
        function add2PtCloud(this, sonarMsg, poseMsg)
            fprintf('INFO : proc mapping : Append to point cloud. \n');
            % scan = rosReadLidarScan(sonarMsg);
            pos = [0, 0, 0];
            quat = [1, 0, 0, 0];
            
            % Getting the sub pose.
            fprintf("INFO : proc mapping : Pose received. \n");
            pos(1) = poseMsg.Position.X;
            pos(2) = poseMsg.Position.Y;
            pos(3) = poseMsg.Position.Z;              

            quat(1) = poseMsg.Orientation.W;
            quat(2) = poseMsg.Orientation.X;
            quat(3) = poseMsg.Orientation.Y;
            quat(4) = poseMsg.Orientation.Z;

            xyzi(:, 1:3) = rosReadXYZ(sonarMsg);
            
            % Temporary swap.
            v = xyzi(:, 1);
            xyzi(:, 1) = xyzi(:, 2);
            xyzi(:, 2) = v;

            xyzi(:, 4) = rosReadField(sonarMsg, 'intensity');

            rowsToDelete = any(xyzi(:,4) < 0.07 & norm(xyzi(:,1:3)) > 5.0, 2);
            xyzi(rowsToDelete, :) = [];

            xyzPoints = zeros([size(xyzi, 1), 3]);
            for i=1:size(xyzi, 1)
                point = sonar2NED(pos.', quat, [0.358, 0, -0.118].', [xyzi(i,1), xyzi(i,2), 0]).'; 
                xyzPoints(i, :) = point(1:3);
            end
            ptCloud = pointCloud(xyzPoints, 'Intensity', xyzi(:, 4));   
            this.bigCloud = pcmerge(this.bigCloud, ptCloud, 0.01);
            
            if coder.target('MATLAB')
                view(this.player, this.bigCloud);
            end
            % this.bigCloud = pcmerge(this.bigCloud, ptCloud, 1);
            % insertPointCloud(this.map3D, [0,0,0,1,0,0,0], ptCloud, double(sonarMsg.RangeMax));
        end
    end

    %% ROS Callbacks
    methods(Static, Access = private)
        function startStopCallback(src, msg)
            Mapper.persistentDataStore('bundleStarted', msg.Data);
            if Mapper.persistentDataStore('bundleStarted')
                fprintf('INFO : proc mapping : Bundle record started \n');
            else
                fprintf('INFO : proc mapping : Bundle record stopped \n');
            end
        end
        
        function sonarCallback(src, msg)
            Mapper.persistentDataStore('newSonarMsg', true);
        end

        function out = persistentDataStore(variable, value)
    
            persistent newSonarMsg bundleStarted;
    
            % Initial variables
            if isempty(newSonarMsg)
                newSonarMsg = false;
            end
    
            if isempty(bundleStarted)
                bundleStarted = false;
            end
    
            if nargin == 1 % GET
                switch true
                    case (strcmpi(variable,'newSonarMsg') == 1)
                        out = newSonarMsg;
                        return
    
                    case (strcmpi(variable,'bundleStarted') == 1)
                        out = bundleStarted;
                        return 
    
                    otherwise
                        out = [];  
                end
    
            elseif nargin == 2 % SET
                switch true
                    case (strcmpi(variable,'newSonarMsg') == 1)
                        newSonarMsg = value;
                        return
        
                    case (strcmpi(variable,'bundleStarted') == 1)
                        bundleStarted = value;
                        return 
                end 
            end
        end
    end
end
