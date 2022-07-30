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
            [labels, numClusters] = pcsegdist(this.filteredPT, this.param.clusterDist);
            
            % Get the biggest point cluster.
            index = mode(labels);
            labels(labels~=index) = 0;
            labels(labels==index) = 1;

            % Extract the cluster.
            hydroPt = select(this.filteredPT, logical(labels));

            if coder.target('MATLAB')
                pcshow(hydroPt);
            end
            
            % Get the center of this cluster.
            center = mean(hydroPt.Location, 1)

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
