classdef PointCloudBundler < handle
    %   PointCloudBundler Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        mBigCloud;
        mPlayer;
        mBundle;
        
        % Filter
        mPreprocessing;

        % Subscribers
        mStartStopSub;
        mClearBundleSub;
        mPoseSub;
        mSonarSub;

        % State
        mLastBundleState;
        mBundleState;
    end
    
    methods
        %% PointCloudBundler Constructor
        function this = PointCloudBundler()   
            if coder.target('MATLAB')
                this.mBigCloud = pointCloud(zeros([1, 3]), 'Intensity', 0);
                this.mPlayer = pcplayer([-20 20],[-20 20],[0 5], 'VerticalAxisDir','Down');
            end
            this.mBundle = zeros(3, 4);
            this.mBundle = zeros(1, 4);

            % Subscribers
            this.mStartStopSub = rossubscriber('/proc_mapping/start_stop', 'std_msgs/Bool', @this.startStopCallback, "DataFormat", "struct");
            this.mClearBundleSub = rossubscriber('/proc_mapping/clear_bundle', 'std_msgs/Bool', @this.clearBundleCallback, "DataFormat", "struct");
            this.mPoseSub = rossubscriber('/proc_nav/auv_pose', 'geometry_msgs/Pose', "DataFormat", "struct");    
            this.mSonarSub = rossubscriber('/provider_sonar/point_cloud2', 'sensor_msgs/PointCloud2', @this.sonarCallback, "DataFormat", "struct");

            this.mLastBundleState = false;

            this.mPreprocessing = Preprocessing();
        end
        %% Step function
        function out = step(this)
            % Verifiy if we just stop the record.
            if this.mLastBundleState && ~this.persistentDataStore('bundleStarted')
                % Record finished.
                this.mLastBundleState = false;
                out = false;
                return;
            else
                % Recording or waiting.
                if this.persistentDataStore('newSonarMsg') && this.persistentDataStore('bundleStarted')
                    this.add2PtCloud(this.mSonarSub.LatestMessage, this.mPoseSub.LatestMessage);
                    this.persistentDataStore('newSonarMsg', false);                  
                end
            end

            % Clear the buffer if requested.
            if this.persistentDataStore('newClearBundleMsg')
                fprintf('INFO : proc mapping : Clearing the bundle \n');
                this.mBundle = zeros(1, 4);
                if coder.target('MATLAB')
                    this.mBigCloud = pointCloud(zeros([1, 3]), 'Intensity', 0);
                    view(this.mPlayer, this.mBigCloud);
                end
                this.persistentDataStore('newClearBundleMsg', false);
            end
            this.mLastBundleState = this.persistentDataStore('bundleStarted');
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

            xyzi = this.mPreprocessing.filter(xyzi);
%             rowsToDelete = any(xyzi(:,4) < 0.07 | sqrt(xyzi(:,1).^2+xyzi(:,2).^2+xyzi(:,3).^2) > 0.1, 2);
%             xyzi(rowsToDelete, :) = [];

            for i=1:size(xyzi, 1)
                point = sonar2NED(pos.', quat, [0.358, 0, -0.118].', [xyzi(i,1), xyzi(i,2), 0]).'; 
                xyzi(i, 1:3) = point(1:3);
            end
            if coder.target('MATLAB')
                ptCloud = pointCloud(xyzi(:, 1:3), 'Intensity', xyzi(:, 4));  
                this.mBigCloud = pcmerge(this.mBigCloud, ptCloud, 0.01);
                view(this.mPlayer, this.mBigCloud);
            end
            this.mBundle = [this.mBundle; xyzi];
        end
        
        %% Getters / Setters
        function out = getBundle(this, lin, col)
            if nargin == 1
                out = this.mBundle;
            elseif nargin == 3
                out = this.mBundle(lin, col);
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

        function clearBundleCallback(src, msg)
            PointCloudBundler.persistentDataStore('newClearBundleMsg', true);
        end

        function out = persistentDataStore(variable, value)
    
            persistent newSonarMsg bundleStarted newClearBundleMsg;
    
            % Initial variables
            if isempty(newSonarMsg)
                newSonarMsg = false;
            end

            if isempty(bundleStarted)
                bundleStarted = false;
            end

            if isempty(newClearBundleMsg)
                newClearBundleMsg = false;
            end
    
            if nargin == 1 % GET
                switch true
                    case (strcmpi(variable,'newSonarMsg') == 1)
                        out = newSonarMsg;
                        return

                    case (strcmpi(variable,'bundleStarted') == 1)
                        out = bundleStarted;
                        return
                        
                    case (strcmpi(variable,'newClearBundleMsg') == 1)
                        out = newClearBundleMsg;
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

                    case (strcmpi(variable,'newClearBundleMsg') == 1)
                        newClearBundleMsg = value;
                        return 
                end 
            end
        end
    end
end
