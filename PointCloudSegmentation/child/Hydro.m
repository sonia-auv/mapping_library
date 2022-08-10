classdef Hydro < PointCloudSegmentation
    % HYDRO Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)        
        % ROS Parameters.
        param;
    end
    %==============================================================================================
    % Public functions
    %==============================================================================================   
    methods
        function this = Hydro(filterPT, param)
            % BUOYS Construct an instance of this class
            %   Detailed explanation goes here
            this@PointCloudSegmentation(filterPT);
            
            this.param = param;
        end
        
        function feature = SegementByAtribute(this, auvPose)    
            if coder.target('MATLAB')
                pcshow(this.filteredPT);
                hold on
            end

            [labels, numClusters] = pcsegdist(this.filteredPT, this.param.clusterDist);


            if coder.target('MATLAB')
                pcshow(this.filteredPT.Location,labels)
                colormap(hsv(numClusters))
            end
            
            % Get the biggest point cluster.
            index = mode(labels);
            labels(labels~=index) = 0;
            labels(labels==index) = 1;

            % Extract the cluster.
            hydroPt = select(this.filteredPT, logical(labels));
            
            % Get the center of this cluster.
            center = mean(hydroPt.Location, 1);
            if coder.target('MATLAB')
                poseplot(quaternion([1, 0, 0, 0]), "Position", center, ScaleFactor=0.2);
            end

            obstacle = rosmessage("sonia_common/ObstacleInfo", "DataFormat", "struct");
            feature = repelem(obstacle, 1);

            obstacle.IsValid = true;
            obstacle.Name = char('Pinger');
            obstacle.Confidence = single(100);
            obstacle.Pose.Position.X = center(1);
            obstacle.Pose.Position.Y = center(2);
            obstacle.Pose.Position.Z = center(3);
            obstacle.Pose.Orientation.W = 1.0;
            obstacle.Pose.Orientation.X = 0.0;
            obstacle.Pose.Orientation.Y = 0.0;
            obstacle.Pose.Orientation.Z = 0.0;
            feature(1) = obstacle;
        end
    end
end
