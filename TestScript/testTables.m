mat = load('Table1.mat');
rawPT = mat.filt;

clusterDist = 0.5;
topAreaMin = 0.6;
topAreaMax = 2.5;
tablesDepth = 1;

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
        goodClusters(i) = 1;
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
    p(3) = 1;
    q = [1, 0, 0, 0];
    poses(i,:) = [p, q];
end

centerPose = mean(poses);
poseplot(quaternion(centerPose(4:7)), "Position", centerPose(1:3), ScaleFactor=0.2);