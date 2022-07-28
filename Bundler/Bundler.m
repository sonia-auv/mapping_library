classdef Bundler < handle

    properties (Access = protected)
        mBigCloud;
        mPlayer;

        % ROS Subcribers
        mStartSub;
        mStopSub;
        mClearBundleSub;
        mPoseSub;

        % State
        mLastBundleState;

        % ROS params
        mParam;

        mBundle;
    end

    methods (Access = public )
        function this = Bundler(param)
            this.mPoseSub = rossubscriber('/proc_nav/auv_states', 'nav_msgs/Odometry', "DataFormat", "struct");
            this.mParam = param;

            % Graphics functions 
            if coder.target('MATLAB')
                this.mBigCloud = pointCloud(zeros([1, 3]), 'Intensity', 0);
                this.mPlayer = pcplayer([-20 20],[-20 20],[0 7], 'VerticalAxisDir','Down', 'MarkerSize', 12);
            end
            
            this.mLastBundleState = false;
        end
      
    end

    methods (Abstract, Access = public )
        out = step(this);
        
    end

    methods (Access = public)
        function clearCloudPlayer(this)
            if coder.target('MATLAB')
                this.mBigCloud = pointCloud(zeros([1, 3]), 'Intensity', 0);
                view(this.mPlayer, this.mBigCloud);
            end
        end

        function bundle = getBundle(this, lin, col)
            if nargin == 1
                bundle = this.mBundle;
            elseif nargin == 3
                bundle = this.mBundle(lin, col);
            end
        end

        function setParam(this, param)
            this.mParam = param;
        end
    end
end
