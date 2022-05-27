load('ptCloud_mur.mat');

% figure('Name', 'Big Cloud');
% pcshow(mapper.bigCloud);

% figure('Name', 'Downsample');
pcdownsamplecloud = pcdownsample(mapper.bigCloud, 'gridAverage', 0.001);
% pcshow(pcdownsamplecloud);

% figure('Name', 'Denoise');
denoiseCloud = pcdenoise(pcdownsamplecloud);
% pcshow(denoiseCloud);

figure('Name', 'Plane1');
[model1, inlierIndices, outlierIndices] = pcfitplane(denoiseCloud, 0.02);

plane1 = select(denoiseCloud, inlierIndices);
remainCloud = select(denoiseCloud, outlierIndices);

[model2, inlierIndices2, outlierIndices2] = pcfitplane(remainCloud, 0.02);
plane2 = select(remainCloud, inlierIndices2);

pcshow(plane1);
hold on 
pcshow(plane2);

plot(model1);
plot(model2);