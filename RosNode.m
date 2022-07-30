classdef RosNode
    %ROSNODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        % ROS Publishers
        outputCloudPublisher;
        obstacleArrayPublisher;

        % ROS parameters
        param;

        % Bundlers
        mPtBundler;
        mScBundler;

        paramUpdateRate;
        rosParamCounter;
        rate;

        obstacleArray;
    end
    
    methods
        %% ROS Node constructor
        function this = RosNode(rate)
            % ROS Publishers
            this.outputCloudPublisher = rospublisher("/proc_mapping/output","sensor_msgs/PointCloud2","DataFormat","struct");
            this.obstacleArrayPublisher = rospublisher("/proc_mapping/obstacle_infos","sonia_common/ObstacleArray","DataFormat","struct", "IsLatching", true);

            obstacle = rosmessage("sonia_common/ObstacleInfo", "DataFormat", "struct");
            this.obstacleArray =  rosmessage("sonia_common/ObstacleArray", "DataFormat", "struct");
            this.obstacleArray.Obstacles = repelem(obstacle, 10).';

            % ROS parameters
            this.param = this.getRosParams();
            this.paramUpdateRate = 2; % seconds
            this.rosParamCounter = 0;
            this.rate = rate;
        end
        
        %% ROS Spinfilt
        function spin(this, spin)
            killNode = false;
            reset(spin);
            fprintf('INFO : proc mapping : Node is started. \n');
            
            % Instances
            this.mPtBundler = PointCloudBundler(this.param);
            this.mScBundler = SoundCloudBundler(this.param);

            while ~killNode
                % PointCloud Bundler
                if ~this.mPtBundler.step()
                    fprintf('INFO : proc mapping : Not bundling. \n');
                    bundle = this.mPtBundler.getBundle();
                    if size(bundle, 1) > 1
                        % Create and filter pointcloud form bundle
                        % Histogram filter
                        histFilter = HistogramFilter(this.param.filter.sonar.histogram_filter);
                        bundle = histFilter.filter(bundle,4); % 3e arg : optional bool debug graph default = false

                        % General filter
                        ptFilter = GeneralFilter(this.param.filter.general);
                        filt = ptFilter.filter(bundle);
    
                        switch upper(this.mPtBundler.getBundleName())
                            case 'BUOYS'
                                buoys = Buoys(filt, this.param.segmentation.buoys);
                                [pose, quat] = this.mPtBundler.getLastSubPose();
                                this.obstacleArray.Obstacles(2:3) = buoys.SegementByAtribute([pose, quat]);
                                send(this.obstacleArrayPublisher, this.obstacleArray);
                        end
                    end
                else
                    % fprintf('INFO : proc mapping : sonar : Bundling or waiting. \n');
                end

                % SoundCloud Bundler
                if ~this.mScBundler.step()
                    scBundle = this.mScBundler.getBundle();
                    if size(scBundle, 1) > 1
                        % Filter the hydrophone point cloud.
                        scFilter = GeneralFilter(this.param.filter.hydro.general);
                        filt = scFilter.filter(scBundle);
                        % Segment the point cloud and find the center.
                        [pose, quat] = this.mPtBundler.getLastSubPose();
                        hydro = Hydro(filt, this.param.segmentation.hydro);
                        this.obstacleArray.Obstacles(6) = hydro.SegementByAtribute([pose, quat]);
                        send(this.obstacleArrayPublisher, this.obstacleArray);
                    end
                else
                    % fprintf('INFO : proc mapping : hydro : Bundling or waiting. \n');
                end

                % Update ROS parameters
                this.rosParamCounter = this.rosParamCounter + 1;
                if this.rosParamCounter >= this.rate * this.paramUpdateRate
                    this.rosParamCounter = 0;
                    this.param = this.getRosParams();
                    this.mPtBundler.setParam(this.param);
                    this.mScBundler.setParam(this.param);
                end            

                waitfor(spin);
            end
        end
    end

    methods (Access = private)
        function param = getRosParams(this)
            % Preprocessing
            param.preprocessing.minIntensity = 0.01;
            param.preprocessing.maxIntensity = 1.0;
            param.preprocessing.minRange = 0.1;
            param.preprocessing.maxRange = 5.0;

            % Filter
            % General
            param.filter.sonar.general.boxSize = 0.05;
            param.filter.hydro.general.boxSize = 0.05;
            param.filter.hydro.freqThreshold = 1000;
            param.filter.general.boxSize = 0.05;
            % Histogram
            param.filter.sonar.histogram_filter.nBin = 100;
            param.filter.sonar.histogram_filter.nBinsToFilterOut = 10;

            % Segmentation
            % Buoys
            param.segmentation.buoys.clusterDist = 0.4;
            param.segmentation.buoys.planeTol = 0.05;
            param.segmentation.buoys.icpInlierRatio = 0.1;
            param.segmentation.buoys.zNormalThres = 0.2;
            param.segmentation.buoys.inPlaneThres = 0.4;
            param.segmentation.buoys.minArea = 0.6;
            param.segmentation.buoys.maxArea = 2.5;
            param.segmentation.buoys.gap = 0.025;

            
            % Hydro
            param.segmentation.hydro.clusterDist = 0.5;

            % Parameters
            % Hydro
            param.parameters.hydro.pingerDepth = 5.0;
            param.parameters.hydro.translation.x = 0.155;
            param.parameters.hydro.translation.y = 0;
            param.parameters.hydro.translation.z = 0.118;

            % Sonar
            param.parameters.sonar.translation.x = 0.358;
            param.parameters.sonar.translation.y = 0;
            param.parameters.sonar.translation.z = -0.118;

            rosparams = RosparamClass(rosparam);
            
            % Preprocessing
            param.preprocessing.minIntensity = rosparams.getValue('/proc_mapping/preprocessing/min_intensity', param.preprocessing.minIntensity);
            param.preprocessing.maxIntensity = rosparams.getValue('/proc_mapping/preprocessing/max_intensity', param.preprocessing.maxIntensity);
            param.preprocessing.minRange = rosparams.getValue('/proc_mapping/preprocessing/min_range', param.preprocessing.minRange);
            param.preprocessing.maxRange = rosparams.getValue('/proc_mapping/preprocessing/max_range', param.preprocessing.maxRange);

            % Filter
            % General
            param.filter.sonar.general.boxSize = rosparams.getValue('/proc_mapping/filter/sonar/general/box_size', param.filter.sonar.general.boxSize);
            param.filter.hydro.general.boxSize = rosparams.getValue('/proc_mapping/filter/hydro/general/box_size', param.filter.hydro.general.boxSize);
            param.filter.hydro.freqThreshold = rosparams.getValue('/proc_mapping/filter/hydro/freq_threshold', param.filter.hydro.freqThreshold);
            % Histogram
            param.filter.sonar.histogram_filter.nBin = rosparams.getValue('/proc_mapping/filter/sonar/histogram_filter/nBin', param.filter.sonar.histogram_filter.nBin);
            param.filter.sonar.histogram_filter.nBinsToFilterOut = rosparams.getValue('/proc_mapping/sonar/filter/histogram_filter/nBinsToFilterOut', param.filter.sonar.histogram_filter.nBinsToFilterOut);

            % Segmentation
            % Buoys
            param.segmentation.buoys.clusterDist = rosparams.getValue('/proc_mapping/segmentation/buoys/cluster_dist', param.segmentation.buoys.clusterDist);
            param.segmentation.buoys.planeTol = rosparams.getValue('/proc_mapping/segmentation/buoys/plane_tol', param.segmentation.buoys.planeTol);
            param.segmentation.buoys.icpInlierRatio = rosparams.getValue('/proc_mapping/segmentation/buoys/icp_inlier_ratio', param.segmentation.buoys.icpInlierRatio);
            param.segmentation.buoys.zNormalThres = rosparams.getValue('/proc_mapping/segmentation/buoys/z_normal_thres', param.segmentation.buoys.zNormalThres);
            param.segmentation.buoys.inPlaneThres = rosparams.getValue('/proc_mapping/segmentation/buoys/in_plane_thres', param.segmentation.buoys.inPlaneThres);
            param.segmentation.buoys.minArea = rosparams.getValue('/proc_mapping/segmentation/buoys/min_area', param.segmentation.buoys.minArea);
            param.segmentation.buoys.maxArea = rosparams.getValue('/proc_mapping/segmentation/buoys/max_area', param.segmentation.buoys.maxArea);
            param.segmentation.buoys.gap = rosparams.getValue('/proc_mapping/segmentation/buoys/gap', param.segmentation.buoys.gap);
        
            % Hydro
            param.segmentation.hydro.clusterDist = rosparams.getValue('/proc_mapping/segmentation/hydro/cluster_dist', param.segmentation.hydro.clusterDist);

            % Parameters
            % Hydro
            param.parameters.hydro.pingerDepth = rosparams.getValue('/proc_mapping/parameters/hydro/pinger_depth', param.parameters.hydro.pingerDepth);
            param.parameters.hydro.translation.x = rosparams.getValue('/proc_mapping/parameters/hydro/translation/x', param.parameters.hydro.translation.x);
            param.parameters.hydro.translation.y = rosparams.getValue('/proc_mapping/parameters/hydro/translation/y', param.parameters.hydro.translation.y);
            param.parameters.hydro.translation.z = rosparams.getValue('/proc_mapping/parameters/hydro/translation/z', param.parameters.hydro.translation.z);

            % Sonar
            param.parameters.sonar.translation.x = rosparams.getValue('/proc_mapping/parameters/sonar/translation/x', param.parameters.sonar.translation.x);
            param.parameters.sonar.translation.y = rosparams.getValue('/proc_mapping/parameters/sonar/translation/y', param.parameters.sonar.translation.y);
            param.parameters.sonar.translation.z = rosparams.getValue('/proc_mapping/parameters/sonar/translation/z', param.parameters.sonar.translation.z);
        end
    end
end

