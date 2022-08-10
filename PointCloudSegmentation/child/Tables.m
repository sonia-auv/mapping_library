classdef Tables < PointCloudSegmentation
    % TABLES Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)        
        % ROS Parameters.
        param;
    end
    %==============================================================================================
    % Public functions
    %==============================================================================================   
    methods
        function this = Tables(filterPT, param)
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
            
            % Separate the pointcloud in different clusters.
            [labels, numClusters] = pcsegdist(this.filteredPT, this.param.clusterDist);

            if coder.target('MATLAB')
                pcshow(this.filteredPT.Location,labels)
                colormap(hsv(numClusters))
            end
            
            % Analyze the clusters.
            goodClusters = zeros(1, numClusters);
            for i = 1:numClusters
                idx = find(labels == i);
                model = pcfitcuboid(this.filteredPT, idx);

                % Calculate top area.
                topArea = model.Dimensions(1) * model.Dimensions(2);

                if topArea >= this.param.topAreaMin && topArea <= this.param.topAreaMax 
                    ratio = min([model.Dimensions(1), model.Dimensions(2)]) / max([model.Dimensions(1), model.Dimensions(2)]);
                    if ratio > this.param.squarenessRatio
                        goodClusters(i) = 1;
                        if coder.target('MATLAB')
                            plot(model)
                        end
                    end
                end
            end
            
            % Extract the pose of each good clusters.
            poses = zeros(sum(goodClusters), 7);          
            for i = 1:sum(goodClusters)
                index = find(goodClusters == 1);
                clusterLabels = labels == index(i);
                clusterPT = select(this.filteredPT, clusterLabels);
                p = zeros(1, 3);
                p(1) = (clusterPT.XLimits(1) + clusterPT.XLimits(2))/2;
                p(2) = (clusterPT.YLimits(1) + clusterPT.YLimits(2))/2;
                p(3) = this.param.poseDepth;
                q = [1, 0, 0, 0];
                poses(i,:) = [p, q];
            end
            
            % Get the center pose.
            centerPose = mean(poses);
            
            if coder.target('MATLAB')
                poseplot(quaternion(centerPose(4:7)), "Position", centerPose(1:3), ScaleFactor=0.2);
            end
            
            % Verify confidence
            confidence = 100;

            % Reduce de confidence if we see more than 3 tables or only one
            % table. 
            Aireif sum(goodClusters) > 3 || sum(goodClusters) == 1
                confidence = 0.25 * confidence;
            end
            % Calculate the distance between two cluster if wee saw 2
            % potential clusters only. 
            if sum(goodClusters) == 2
                distance = pdist([poses(1, 1:3);poses(2, 1:3)]);
                if distance > this.param.maxBetweenDist
                    confidence = 0.25 * confidence;
                end
            end 
            % Check if the 3 tables are aligned if we exactly 3 tables.
            if sum(goodClusters) == 3
                % Vector 1
                v1 = poses(2, 1:2) - poses(1, 1:2);
                v2 = poses(3, 1:2) - poses(1, 1:2);
                angle = subspace(v1.', v2.');
                if angle > 0.4
                    confidence = 0.5 * confidence;
                end
            end

            % Return the obstacle info.
            obstacle = rosmessage("sonia_common/ObstacleInfo", "DataFormat", "struct");
            feature = repelem(obstacle, 1);

            obstacle.IsValid = true;
            obstacle.Name = char('Tables');
            obstacle.Confidence = single(confidence);
            obstacle.Pose.Position.X = centerPose(1);
            obstacle.Pose.Position.Y = centerPose(2);
            obstacle.Pose.Position.Z = centerPose(3);
            obstacle.Pose.Orientation.W = 1.0;
            obstacle.Pose.Orientation.X = 0.0;
            obstacle.Pose.Orientation.Y = 0.0;
            obstacle.Pose.Orientation.Z = 0.0;
            feature(1) = obstacle;
        end
    end
end
