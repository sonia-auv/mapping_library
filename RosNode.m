classdef RosNode
    %ROSNODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        % ROS Publishers
        outputCloudPublisher;
        outputPosePublisher;

        % ROS parameters
        param;

        mPtBundler;

        paramUpdateRate;
        counter;
        rate;
    end
    
    methods
        %% ROS Node constructor
        function this = RosNode(rate)
            % ROS Publishers
            this.outputCloudPublisher = rospublisher("/proc_mapping/output","sensor_msgs/PointCloud2","DataFormat","struct");
            this.outputPosePublisher = rospublisher("/proc_mapping/output_pose","sonia_common/AddPose","DataFormat","struct");

            % ROS parameters
            this.param = this.getRosParams();
            
            this.paramUpdateRate = 2; % seconds
            this.counter = 0;
            this.rate = rate
        end
        
        %% ROS Spinfilt
        function spin(this, spin)
            killNode = false;
            reset(spin);
            fprintf('INFO : proc mapping : Node is started. \n');
            fprintf('INFO : proc mapping : Wait for point cloud. \n');
            
            % Instances
            this.mPtBundler = PointCloudBundler(this.param);
            ptFilter = GeneralFilter();

            while ~killNode
                if ~this.mPtBundler.step()
                    fprintf('INFO : proc mapping : Not bundling. \n');
                    bundle = this.mPtBundler.getBundle();
                    if size(bundle, 1) > 1

                        % Create and filter pointcloud form bundle
                        filt = ptFilter.filter(bundle);
                        
                        buoys = Buoys(filt, this.param.segmentation.buoys);
                        output = buoys.SegementByAtribute();
                        
                        %pack = packagePointCloud(single(output.Location), single(output.Intensity));
                        %send(this.outputCloudPublisher, pack);
                    end
                else
                    % fprintf('INFO : proc mapping : Bundling or waiting. \n');
                end

                this.counter = this.counter + 1;
                if this.counter >= this.rate * this.paramUpdateRate
                    this.counter = 0;
                    this.param = this.getRosParams();
                    this.mPtBundler.setParam(this.param);
                end

                waitfor(spin);
            end
        end
    end

    methods (Access = private)
        function param = getRosParams(this)
            % Preprocessing
            param.preprocessing.minIntensity = 0.1;
            param.preprocessing.maxIntensity = 1.0;
            param.preprocessing.minRange = 0.1;
            param.preprocessing.maxRange = 5.0;

            % Segmentation
            % Buoys
            param.segmentation.buoys.clusterDist = 0.4;
            param.segmentation.buoys.planeTol = 0.02;
            param.segmentation.buoys.icpInlierRatio = 0.1;
            param.segmentation.buoys.zNormalThres = 0.2;
            param.segmentation.buoys.inPlaneThres = 0.4;
            param.segmentation.buoys.minArea = 0.6;
            param.segmentation.buoys.maxArea = 2.5;

            rosparams = RosparamClass(rosparam);
            
            % Preprocessing
            param.preprocessing.minIntensity = rosparams.getValue('/proc_mapping/preprocessing/min_intensity', param.preprocessing.minIntensity);
            param.preprocessing.maxIntensity = rosparams.getValue('/proc_mapping/preprocessing/max_intensity', param.preprocessing.maxIntensity);
            param.preprocessing.minRange = rosparams.getValue('/proc_mapping/preprocessing/min_range', param.preprocessing.minRange);
            param.preprocessing.maxRange = rosparams.getValue('/proc_mapping/preprocessing/max_range', param.preprocessing.maxRange);

            % Segmentation
            % Buoys
            param.segmentation.buoys.clusterDist = rosparams.getValue('/proc_mapping/segmentation/buoys/cluster_dist', param.segmentation.buoys.clusterDist);
            param.segmentation.buoys.planeTol = rosparams.getValue('/proc_mapping/segmentation/buoys/plane_tol', param.segmentation.buoys.planeTol);
            param.segmentation.buoys.icpInlierRatio = rosparams.getValue('/proc_mapping/segmentation/buoys/icp_inlier_ratio', param.segmentation.buoys.icpInlierRatio);
            param.segmentation.buoys.zNormalThres = rosparams.getValue('/proc_mapping/segmentation/buoys/z_normal_thres', param.segmentation.buoys.zNormalThres);
            param.segmentation.buoys.inPlaneThres = rosparams.getValue('/proc_mapping/segmentation/buoys/in_plane_thres', param.segmentation.buoys.inPlaneThres);
            param.segmentation.buoys.minArea = rosparams.getValue('/proc_mapping/segmentation/buoys/min_area', param.segmentation.buoys.minArea);
            param.segmentation.buoys.maxArea = rosparams.getValue('/proc_mapping/segmentation/buoys/max_area', param.segmentation.buoys.maxArea);
        end
    end
end

