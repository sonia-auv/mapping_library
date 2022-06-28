load('ptCloud_filt.mat');

filteredPT = filt;

ptcloud=pcread('buoy.ply');
pcshow(ptcloud)



[modelCyl1, inlierIndices, outlierIndices] = pcfitcylinder(filteredPT, 0.05, [0,0,1], 4);
cyl1 = select(filteredPT, inlierIndices);
remainCloud = select(filteredPT, outlierIndices);

[model1, inlierIndices1, outlierIndices1] = pcfitplane(remainCloud, 0.02, [1,0,0], 45);
plane1 = select(remainCloud, inlierIndices1);
remainCloud = select(remainCloud, outlierIndices1);

% [model2, inlierIndices2, outlierIndices2] = pcfitplane(remainCloud, 0.02, [1,0,0], 45);
% plane2 = select(remainCloud, inlierIndices2);

% Extraire les point orienté
[p1, q1] = getOrientedPointOnPlanarFace(model1, plane1);
% [p2, q2] = getOrientedPointOnPlanarFace(model2, plane2);

figure('Name', 'Plane1');
pcshow(plane1);
hold on 
pcshow(filteredPT);
% pcshow(plane2);

plot(model1);
% plot(model2);
plot(modelCyl1);

poseplot(quaternion(q1),'Position',p1);
% poseplot(quaternion(q2),'Position',p2);

function [p,q] = getOrientedPointOnPlanarFace(model, plane)

    % vecteur initial
    v1 =[1,0,0];
    
    % Trouver la transformaion angulaire du plan
    q = quatUtilities.quaternionForm2Vectors(v1, model.Normal);
    
    % Trouver la transformation linéaire du plan
    p = zeros(1,3);
    p(1) = (plane.XLimits(2)-plane.XLimits(1))/2 + plane.XLimits(1);
    p(2) = (plane.YLimits(2)-plane.YLimits(1))/2 + plane.YLimits(1);
    p(3) = (plane.ZLimits(2)-plane.ZLimits(1))/2 + plane.ZLimits(1);
end