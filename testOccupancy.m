load('ptCloud_mur.mat');

map3D = occupancyMap3D(20);

insertPointCloud(map3D, [0,0,0,1,0,0,0], mapper.bigCloud, 20);

show(map3D);