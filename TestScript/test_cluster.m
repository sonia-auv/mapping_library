load('MAT/torpille_filt.mat');
filteredPT = filt;

% Get cluster
[labels,numClusters] = pcsegdist(filteredPT,0.4);

pcshow(filteredPT.Location,labels)
colormap(hsv(numClusters))

for i=0; 