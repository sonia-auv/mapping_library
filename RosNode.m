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
            wCorner = WallCorner();

            while ~killNode
                if ~ptBundler.step()
                    fprintf('INFO : proc mapping : Not bundling. \n');
                    bundle = ptBundler.getBundle();
                    if size(bundle, 1) > 1
                        filt = ptFilter.filter(bundle);
                        outputPose = wCorner.SegementByAtribute(filt);
                        
                        rotation = quat2eul(outputPose(1:4));
                        position = outputPose(5:7);
                        
                        pose = rosmessage('sonia_common/AddPose',"DataFormat","struct");
                        disp(position);
                        pose.Position.X = position(1);
                        pose.Position.Y = position(2);
                        pose.Position.Z = position(3);
                        disp(rotation);
                        pose.Orientation.X = rotation(1);
                        pose.Orientation.Y = rotation(2);
                        pose.Orientation.Z = rotation(3);
                        send(this.outputPosePublisher, pose);
                        % pack = packagePointCloud(single(output.Location), single(output.Intensity));
                        % send(this.outputCloudPublisher, pack);
                    end
                else
                    % fprintf('INFO : proc mapping : Bundling or waiting. \n');
                end
                waitfor(spin);
            end
        end
    end
end

