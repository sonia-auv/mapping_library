%load('ptCloud_filt.mat');
load('MAT/torpille_filt.mat');

filteredPT = filt;

clusterDist = 0.4;
planeTol = 0.02;

%preselect;
zNormalThres = 0.2;
inPlaneThres = 0.40;
areaThres = [0.6 2.5];

tic;
% Import buoy point cloud.
buoy=pcdownsample(pcread('buoy.ply'),'gridAverage',0.01);

% Get clusters
[labels,numClusters] = pcsegdist(filteredPT,clusterDist);
goodCluster = zeros(1, numClusters);
for i =1  : numClusters

    % extract pointCloud
        clusterLabels = labels == i;
        clusterPT = select(filteredPT,clusterLabels);

    % fit plane on cluster
        [model, indexOnPlane, indexOffPlane, meanError ] = pcfitplane(clusterPT, planeTol );

        plane =  select(clusterPT, indexOnPlane);

    % Get Z normal
        zNormal = model.Normal(3);

    % Ratio in plane
        percentInPlane = max(size(indexOnPlane)) / clusterPT.Count;

    % Extract pose of the plane.
        [p, q] = getOrientedPointOnPlanarFace(model, plane);


      box = objectAllignBoudingBox(q, plane,model);
      area = box(2)*box(3);

      if (zNormal < zNormalThres && percentInPlane > inPlaneThres && area > areaThres(1) && area < areaThres(2))

        goodCluster(i) = 1;
        figure('Name', strcat('Plane1',char(i)));
        pcshow(plane);
        hold on 
        
      end
end

switch sum(goodCluster)

    % Suspect 2 buyos in the same clusters
    case 1
        index = find(goodCluster==1);
        clusterLabels = labels == index;
        PT =  select(filteredPT, clusterLabels);
        
        kmeansIndex = kmeans(PT.Location,2);
        pcshow(PT.Location,kmeansIndex)
        colormap(hsv(2))
        
        %for each buoys
        for i = 1 : 2 
           subPT =  select(PT, kmeansIndex == i);  
           [model, indexOnPlane, indexOffPlane, meanError ] = pcfitplane(subPT, planeTol );
           plane =  select(subPT, indexOnPlane);

           [p,q] = getOrientedPointOnPlanarFace(model, subPT);

           % Transform the buoy on the plane.
           tformRansac = rigid3d(quat2rotm(quatinv(q)), p);
           buoyTformed = pctransform(buoy, tformRansac );
           

           % run icp
             tformICP = pcregistericp(buoyTformed , plane,"InlierRatio",0.1);
             icpTrans = rigid3d(tformRansac.T * tformICP.A.');

             buoyTformed = pctransform(buoy, icpTrans);
             pcshow(buoyTformed)

             poseplot(quaternion(rotm2quat(icpTrans.Rotation.')),'Position',icpTrans.Translation,ScaleFactor=0.1);

        end
    otherwise
end

pcshow(filteredPT);
toc;

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

function  box = objectAllignBoudingBox(q, pc,model)

    tRot = quat2rotm(q);
    tf = rigid3d(tRot, [0 0 0]);
    pct = pctransform(pc, tf);
    %pcshow(pct);
    box = zeros(1,3);
    box(1) = (pct.XLimits(2)-pct.XLimits(1));
    box(2) = (pct.YLimits(2)-pct.YLimits(1));
    box(3) = (pct.ZLimits(2)-pct.ZLimits(1));
    

end