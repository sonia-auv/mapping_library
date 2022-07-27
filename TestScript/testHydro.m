

% hydro =[0.150, 0, 0.118]
% 
% q = quatinv(eul2quat(deg2rad([180,0,90]),"ZYX"))
% 
% quatrotate(q,hydro)


load('testHydro.mat')
rawPT = pointCloud(hydroo);

filteredPT = pcdenoise(pcdownsample(rawPT, 'gridAverage', 0.07));

[labels, numClusters] = pcsegdist(filteredPT, 0.5);
% pcshow(filteredPT.Location,labels)
% colormap(hsv(numClusters))

index = mode(labels);
labels(labels~=index) = 0;
labels(labels==index) = 1;

hydroPt = select(filteredPT, logical(labels));

pcshow(hydroPt);

center = mean(hydroPt.Location, 1)
