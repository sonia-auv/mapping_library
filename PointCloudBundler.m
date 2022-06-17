classdef PointCloudBundler < handle
    %   PointCloudBundler Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        bigCloud;
        player;
        bundle;

        % Subscribers
        startStopSub;
        poseSub;
        sonarSub;

        % State
        lastBundleState;
        bundleState;
    end
    
    methods
        %% PointCloudBundler Constructor
        function this = PointCloudBundler()   
            if coder.target('MATLAB')
                this.bigCloud = pointCloud(zeros([1, 3]), 'Intensity', 0);
                this.player = pcplayer([-20 20],[-20 20],[0 5], 'VerticalAxisDir','Down');
            end
            this.bundle = zeros(3, 4);
            this.bundle = zeros(1, 4);

            % Subscribers
            this.startStopSub = rossubscriber('/proc_mapping/start_stop', 'std_msgs/Bool', @this.startStopCallback, "DataFormat", "struct");
            this.poseSub = rossubscriber('/proc_nav/auv_pose', 'geometry_msgs/Pose', "DataFormat", "struct");    
            this.sonarSub = rossubscriber('/provider_sonar/point_cloud2', 'sensor_msgs/PointCloud2', @this.sonarCallback, "DataFormat", "struct");
        
            this.lastBundleState = false;
        end
        %% Step function
        function out = step(this)
            if this.lastBundleState && ~this.persistentDataStore('bundleStarted')
                % Record finished.
                this.lastBundleState = false;
                out = false;
                return;
            else
                % Recording or waiting.
                if this.persistentDataStore('newSonarMsg') && this.persistentDataStore('bundleStarted')
                    this.add2PtCloud(this.sonarSub.LatestMessage, this.poseSub.LatestMessage);
                    this.persistentDataStore('newSonarMsg', false);
                end
            end
            this.lastBundleState = this.persistentDataStore('bundleStarted');
            out = true;
            return;
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
            
            xyzi = zeros(sonarMsg.Width, 4);
            xyzi(:, 1:3) = rosReadXYZ(sonarMsg);
            
            % Temporary swap.
%             v = xyzi(:, 1);
%             xyzi(:, 1) = xyzi(:, 2);
%             xyzi(:, 2) = v;

            xyzi(:, 4) = rosReadField(sonarMsg, 'intensity');

            rowsToDelete = any(xyzi(:,4) < 0.07 & sqrt(xyzi(:,1).^2+xyzi(:,2).^2+xyzi(:,3).^2) > 0.1, 2);
            xyzi(rowsToDelete, :) = [];

            for i=1:size(xyzi, 1)
                point = sonar2NED(pos.', quat, [0.358, 0, -0.118].', [xyzi(i,1), xyzi(i,2), 0]).'; 
                xyzi(i, 1:3) = point(1:3);
            end
            if coder.target('MATLAB')
                ptCloud = pointCloud(xyzi(:, 1:3), 'Intensity', xyzi(:, 4));  
                this.bigCloud = pcmerge(this.bigCloud, ptCloud, 0.01);
                view(this.player, this.bigCloud);
            end
            this.bundle = [this.bundle; xyzi];
        end
        
        %% Getters / Setters
        function out = getBundle(this, lin, col)
            if nargin == 1
                out = this.bundle;
            elseif nargin == 3
                out = this.bundle(lin, col);
            end
        end
    end

    %% ROS Callbacks
    methods(Static, Access = private)
        function sonarCallback(src, msg)
            PointCloudBundler.persistentDataStore('newSonarMsg', true);
        end

        function startStopCallback(src, msg)
            PointCloudBundler.persistentDataStore('bundleStarted', msg.Data);
            if PointCloudBundler.persistentDataStore('bundleStarted')
                fprintf('INFO : proc mapping : Bundle record started \n');
            else
                fprintf('INFO : proc mapping : Bundle record stopped \n');
            end
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
