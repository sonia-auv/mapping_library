classdef RosNode
    %ROSNODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        outputCloudPublisher;
        outputPosePublisher;
    end
    
    methods
        %% ROS Node constructor
        function this = RosNode()
            this.outputCloudPublisher = rospublisher("/proc_mapping/output","sensor_msgs/PointCloud2","DataFormat","struct");
            this.outputPosePublisher = rospublisher("/proc_mapping/output_pose","sonia_common/AddPose","DataFormat","struct");
        end
        
        %% ROS Spin
        function spin(this, spin)
            killNode = false;
            reset(spin);
            fprintf('INFO : proc mapping : Node is started. \n');
            fprintf('INFO : proc mapping : Wait for point cloud. \n');
            
            % Instances
            ptBundler = PointCloudBundler();
            ptFilter = GeneralFilter();

            while ~killNode
                if ~ptBundler.step()
                    fprintf('INFO : proc mapping : Not bundling. \n');
                    bundle = ptBundler.getBundle();
                    if size(bundle, 1) > 1

                        % Create and filter pointcloud form bundle
                        filt = ptFilter.filter(bundle);
                        
                        buoys = Buoys(filt);
                        output = buoys.SegementByAtribute();
                        
                        %pack = packagePointCloud(single(output.Location), single(output.Intensity));
                        %send(this.outputCloudPublisher, pack);
                    end
                else
                    % fprintf('INFO : proc mapping : Bundling or waiting. \n');
                end
                waitfor(spin);
            end
        end
    end
end

