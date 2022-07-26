classdef SoundCloudBundler < Bundler
    %   SoundCloudBundler Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        % Subscribers
        mHydroSub;
        hydroPose = [.155, 0, .118];
        
    end
    
    properties
        i;
    end

    methods (Access = public)
        %% SoundCloudBundler Constructor
        function this = SoundCloudBundler(param) 
            this@Bundler(param); 

            this.mBundle = zeros(1, 4);

            % Subscribers
            this.mHydroSub = rossubscriber('/proc_hydrophone/ping', 'sonia_common/PingAngles', @this.hydroCallback, "DataFormat", "struct");
            this.mStartSub = rossubscriber('/proc_mapping/hydro/start', 'std_msgs/Bool', @this.startCallback, "DataFormat", "struct");
            this.mStopSub = rossubscriber('/proc_mapping/hydro/stop', 'std_msgs/Bool', @this.stopCallback, "DataFormat", "struct");
            this.mClearBundleSub = rossubscriber('/proc_mapping/hydro/clear_bundle', 'std_msgs/Bool', @this.clearBundleCallback, "DataFormat", "struct"); 
            this.i=1;  
        end

        %% Step function
        function out = step(this)
            % Verifiy if we just stop the record.
            if this.mLastBundleState && ~this.persistentDataStore('bundleStarted')
                % Record finished.
                this.mLastBundleState = false;
                out = false;
                fprintf('INFO : proc mapping : hydro : Record finished. \n');
                return;
            else
                % Recording or waiting.
                if this.persistentDataStore('newHydroMsg') && this.persistentDataStore('bundleStarted')
                    if ~isempty(this.mPoseSub.LatestMessage)
                        this.add2PtCloud(this.mHydroSub.LatestMessage, this.mPoseSub.LatestMessage.Pose.Pose);
                    end
                    this.persistentDataStore('newHydroMsg', false);                  
                end
            end

            % Clear the buffer if requested.
            if this.persistentDataStore('newClearBundleMsg')
                fprintf('INFO : proc mapping : hydro : Clearing the sonar bundle \n');
                this.mBundle = zeros(1, 4);
                this.clearCloudPlayer();
                this.persistentDataStore('newClearBundleMsg', false);
            end
            this.mLastBundleState = this.persistentDataStore('bundleStarted');
            out = true;
            return;
        end


    end

    %==============================================================================================
    % Privates functions
    %==============================================================================================
    methods(Access = private)
        % Adding to the point cloud.
        function add2PtCloud(this, hydroMsg, poseMsg)
            if coder.target('MATLAB')
                fprintf('INFO : proc mapping : hydro : Append to point cloud. \n');
            end
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

            %fix
            quat = quatinv(quat);
            
            xyzi = zeros(1, 4);
            hydro = this.hydroAngle2Cartesian(hydroMsg.Heading, hydroMsg.Elevation, poseMsg);
            

            point = sonar2NED(pos.', quat, this.hydroPose.', hydro.').'; 
            xyzi(1, 1:3) = point(1:3);
            xyzi(4) = hydroMsg.Snr; % 
            
            xyzi
            this.i = this.i + 1;

            if coder.target('MATLAB')
                ptCloud = pointCloud(xyzi(:, 1:3), 'Intensity', xyzi(:, 4));

                this.mBigCloud = pcmerge(this.mBigCloud, ptCloud, 0.01);
                view(this.mPlayer, this.mBigCloud);
            end
            this.mBundle = [this.mBundle; xyzi];
        end  
        
        function pose = hydroAngle2Cartesian(this, phi, theta, poseMsg)

            quat(1) = poseMsg.Orientation.W;
            quat(2) = poseMsg.Orientation.X;
            quat(3) = poseMsg.Orientation.Y;
            quat(4) = poseMsg.Orientation.Z;

            % Trouver la valeur de z
            lever = quatrotate(quatinv(quat),this.hydroPose);
            z = this.param.parameters.hydro.pingerDepth -  poseMsg.Position.Z  + lever(3);

            rho = z/cos(theta);

            pose = [rho *cos(phi)*sin(theta);
                    rho * sin(phi) * sin(theta);
                    z];
        end
    end
    
    %==============================================================================================
    %% ROS Callbacks
    %==============================================================================================
    methods(Static, Access = private)
        function hydroCallback(src, msg)
            SoundCloudBundler.persistentDataStore('newHydroMsg', true);
        end

        function startCallback(src, msg)
            SoundCloudBundler.persistentDataStore('bundleStarted', true);
            fprintf('INFO : proc mapping : hydro : Bundle record started \n');
        end

        function stopCallback(src, msg)
            SoundCloudBundler.persistentDataStore('bundleStarted', false);
            fprintf('INFO : proc mapping : hydro : Bundle record stopped \n');
        end

        function clearBundleCallback(src, msg)
            SoundCloudBundler.persistentDataStore('newClearBundleMsg', true);
        end

        function out = persistentDataStore(variable, value)

            persistent newHydroMsg bundleStarted newClearBundleMsg;
    
            % Initial variables
            if isempty(newHydroMsg)
                newHydroMsg = false;
            end
            if isempty(bundleStarted)
                bundleStarted = false;
            end
            if isempty(newClearBundleMsg)
                newClearBundleMsg = false;
            end
    
            if nargin == 1 % GET
                switch true
                    case (strcmpi(variable,'newHydroMsg') == 1)
                        out = newHydroMsg;
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
                    case (strcmpi(variable,'newHydroMsg') == 1)
                        newHydroMsg = value;
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
