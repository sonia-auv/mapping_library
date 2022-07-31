mat = load('Table3.mat');
rawPT = mat.filt;

clusterDist = 0.5;
topAreaMin = 0.6;
topAreaMax = 2.5;
poseDepth = 1;
maxBetweenDist = 4.0;
maxBetweenAngle = 0.3;  % rad

% Filtered point cloud.
filteredPT = pcdenoise(pcdownsample(rawPT, 'gridAverage', 0.07));

pcshow(filteredPT);
hold on

% Separate the pointcloud in different clusters.
[labels, numClusters] = pcsegdist(filteredPT, clusterDist);
pcshow(filteredPT.Location,labels)
colormap(hsv(numClusters))

goodClusters = zeros(1, numClusters);

% Analyze the clusters.
for i = 1:numClusters
    idx = find(labels == i);
    model = pcfitcuboid(filteredPT,idx);
    plot(model)

    % Calculate top area.
    topArea = model.Dimensions(1) * model.Dimensions(2);
    model.Dimensions
    if topArea >= topAreaMin && topArea <= topAreaMax 
        ratio = min([model.Dimensions(1), model.Dimensions(2)]) / max([model.Dimensions(1), model.Dimensions(2)]);
        if ratio > 0.3
            goodClusters(i) = 1;
        end
    end
end

poses = zeros(sum(goodClusters), 7);

for i = 1:sum(goodClusters)
    index = find(goodClusters == 1);
    clusterLabels = labels == index(i);
    clusterPT = select(filteredPT, clusterLabels);
    p = zeros(1, 3);
    p(1) = (clusterPT.XLimits(1) + clusterPT.XLimits(2))/2;
    p(2) = (clusterPT.YLimits(1) + clusterPT.YLimits(2))/2;
    p(3) = poseDepth;
    q = [1, 0, 0, 0];
    poses(i,:) = [p, q];
end

centerPose = mean(poses);
poseplot(quaternion(centerPose(4:7)), "Position", centerPose(1:3), ScaleFactor=0.2);

% Verify confidence
confidence = 100;

% Reduce de confidence if we see more than 3 tables or only one
% table. 
if sum(goodClusters) > 3 || sum(goodClusters) == 1
    confidence = 0.25 * confidence;
end
% Calculate the distance between two cluster if wee saw 2
% potential clusters only. 
if sum(goodClusters) == 25
    distance = pdist([poses(1, 1:3);poses(2, 1:3)]);
    if distance > maxBetweenDist
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