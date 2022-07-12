classdef PointCloudBundler < Bundler
    %   PointCloudBundler Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        % Filter
        mPreprocessing;

        % Subscribers
        mSonarSub;  
    end
    
    methods (Access = public)
        %% PointCloudBundler Constructor
        function this = PointCloudBundler(param) 
            this@Bundler(param); 

            this.mBundle = zeros(3, 4);
            this.mBundle = zeros(1, 4);

            % Subscribers
            this.mStartSub = rossubscriber('/proc_mapping/sonar/start', 'std_msgs/String', @this.startCallback, "DataFormat", "struct");
            this.mStopSub = rossubscriber('/proc_mapping/sonar/stop', 'std_msgs/Bool', @this.stopCallback, "DataFormat", "struct");
            this.mClearBundleSub = rossubscriber('/proc_mapping/sonar/clear_bundle', 'std_msgs/Bool', @this.clearBundleCallback, "DataFormat", "struct");   
            this.mSonarSub = rossubscriber('/provider_sonar/point_cloud2', 'sensor_msgs/PointCloud2', @this.sonarCallback, "DataFormat", "struct");

            this.mPreprocessing = Preprocessing(param.preprocessing);
        end

        %% Step function
        function out = step(this)
            % Verifiy if we just stop the record.
            if this.mLastBundleState && ~this.persistentDataStore('bundleStarted')
                % Record finished.
                this.mLastBundleState = false;
                out = false;
                fprintf('INFO : proc mapping : sonar : Record finished. \n');
                return;
            else
                % Recording or waiting.
                if this.persistentDataStore('newSonarMsg') && this.persistentDataStore('bundleStarted')
                    this.add2PtCloud(this.mSonarSub.LatestMessage, this.mPoseSub.LatestMessage.Pose.Pose);
                    this.persistentDataStore('newSonarMsg', false);                  
                end
            end

            % Clear the buffer if requested.
            if this.persistentDataStore('newClearBundleMsg')
                fprintf('INFO : proc mapping : sonar : Clearing the sonar bundle \n');
                this.mBundle = zeros(1, 4);
                this.clearCloudPlayer();
                this.persistentDataStore('newClearBundleMsg', false);
            end
            this.mLastBundleState = this.persistentDataStore('bundleStarted');
            out = true;
            return;
        end

        %% Getters / Setters
        function name = getBundleName(this)
            name = '';
            if ~isempty(this.mStartSub.LatestMessage)
                name = this.mStartSub.LatestMessage.Data;
            end
        end

        function [p, q] = getLastSubPose(this)
            p = zeros(1, 3);
            q = zeros(1, 4);
            if ~isempty(this.mPoseSub.LatestMessage)
                p = [this.mPoseSub.LatestMessage.Pose.Pose.Position.X, ...
                     this.mPoseSub.LatestMessage.Pose.Pose.Position.Y, ...
                     this.mPoseSub.LatestMessage.Pose.Pose.Position.Z];
                q = [this.mPoseSub.LatestMessage.Pose.Pose.Orientation.W, ...
                    this.mPoseSub.LatestMessage.Pose.Pose.Orientation.X, ...
                    this.mPoseSub.LatestMessage.Pose.Pose.Orientation.Y, ...
                    this.mPoseSub.LatestMessage.Pose.Pose.Orientation.Z];
            end
        end

        function setParam(this, param)
            this.setParam@Bundler(param);
            this.mPreprocessing.setParam(param.preprocessing);
        end
    end

    %==============================================================================================
    % Privates functions
    %==============================================================================================
    methods(Access = private)
        %% Adding to the point cloud.
        function add2PtCloud(this, sonarMsg, poseMsg)
            fprintf('INFO : proc mapping : sonar : Append to point cloud. \n');
            % scan = rosReadLidarScan(sonarMsg);
            pos = [0, 0, 0];
            quat = [1, 0, 0, 0];
            
            % Getting the sub pose.
            pos(1) = poseMsg.Position.X;
            pos(2) = poseMsg.Position.Y;
            pos(3) = poseMsg.Position.Z;              

            quat(1) = poseMsg.Orientation.W;
            quat(2) = poseMsg.Orientation.X;
            quat(3) = poseMsg.Orientation.Y;
            quat(4) = poseMsg.Orientation.Z;

            quat = quatinv(quat);
            
            xyzi = zeros(sonarMsg.Width, 4);
            xyzi(:, 1:3) = rosReadXYZ(sonarMsg);

            xyzi(:, 4) = rosReadField(sonarMsg, 'intensity');

            xyzi = this.mPreprocessing.filter(xyzi);

            tx = this.mParam.parameters.sonar.translation.x;
            ty = this.mParam.parameters.sonar.translation.y;
            tz = this.mParam.parameters.sonar.translation.z;

            for i=1:size(xyzi, 1)
                point = sonar2NED(pos.', quat, [tx, ty, tz].', [xyzi(i,1), xyzi(i,2), 0]).'; 
                xyzi(i, 1:3) = point(1:3);
            end
            if coder.target('MATLAB')
                ptCloud = pointCloud(xyzi(:, 1:3), 'Intensity', xyzi(:, 4));

                this.mBigCloud = pcmerge(this.mBigCloud, ptCloud, 0.01);
                view(this.mPlayer, this.mBigCloud);
            end
            this.mBundle = [this.mBundle; xyzi];
        end     
    end
    %==============================================================================================
    %% ROS Callbacks
    %==============================================================================================
    methods(Static, Access = private)
        function sonarCallback(src, msg)
            PointCloudBundler.persistentDataStore('newSonarMsg', true);
        end

        function startCallback(src, msg)
            PointCloudBundler.persistentDataStore('bundleStarted', true);
            fprintf('INFO : proc mapping : sonar : Bundle record started \n');
        end

        function stopCallback(src, msg)
            PointCloudBundler.persistentDataStore('bundleStarted', false);
            fprintf('INFO : proc mapping : sonar : Bundle record stopped \n');
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