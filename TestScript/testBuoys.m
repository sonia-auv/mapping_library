load('ptCloud_filt.mat');
filteredPT = filt;

% Import buoy point cloud.
buoy=pcread('buoy.ply');

% Rotation of the buoy.
rot = rotx(180);
transBuoy = rigid3d(rot, [0 0 0]);
buoy = pctransform(buoy, transBuoy);
% pcshow(buoy)

% Remove a cylinder feature.
[modelCyl1, inlierIndices, outlierIndices] = pcfitcylinder(filteredPT, 0.05, [0,0,1], 4);
cyl1 = select(filteredPT, inlierIndices);
remainCloud = select(filteredPT, outlierIndices);

% Find a plane in the point cloud.
[model1, inlierIndices1, outlierIndices1] = pcfitplane(remainCloud, 0.02, [1,0,0], 45);
plane1 = select(remainCloud, inlierIndices1);
remainCloud = select(remainCloud, outlierIndices1);

% [model2, inlierIndices2, outlierIndices2] = pcfitplane(remainCloud, 0.02, [1,0,0], 45);
% plane2 = select(remainCloud, inlierIndices2);

% Extract pose of the plane.
[p1, q1] = getOrientedPointOnPlanarFace(model1, plane1);
% [p2, q2] = getOrientedPointOnPlanarFace(model2, plane2);

% Translate the buoy on the plane.
pose = rigid3d(quat2rotm(q1), [0 0 0]);
tform1 = cameraPoseToExtrinsics(pose);
buoyTformed = pctransform(buoy, tform1);
a = [1 0 0 0; ...
     0 1 0 0; ...
     0 0 1 0; ...
     p1(1), p1(2), p1(3), 1];
trans = rigid3d(a);
buoyTformed = pctransform(buoyTformed, trans);

% Transform the buoy to the best spot that represent the buoy in the plane
% point cloud.

% Using ICP.
tform = pcregistericp(buoyTformed, plane1, 'Extrapolate', false);

% Transformation.
buoyTformed = pctransform(buoyTformed, tform);

% Bragging with some graphs.
figure('Name', 'Plane1');
pcshow(plane1);
hold on 
pcshow(filteredPT);
pcshow(buoyTformed)
% pcshow(plane2);

plot(model1);
% plot(model2);
% plot(modelCyl1);

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