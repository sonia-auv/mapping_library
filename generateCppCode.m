% This Script generate the c++ code for proc_planner.
% Make sure to run this script in src/proc_planner folder.
    
    clear;
    
% add shared lib folder   
%     addpath("../shared_libraries/");

% Set the code generation config
    cfg = coder.config('exe');
    cfg.TargetLang = "C++";
    cfg.GenCodeOnly = true;
    cfg.Hardware = coder.hardware('Robot Operating System (ROS)');
    cfg.BuildConfiguration = "Faster Runs";
    cfg.Hardware.BuildAction = 'none';

% Generate the code
    codegen proc_mapping -v -args {} -config cfg

% Load build Info mat
    load('./codegen/exe/proc_mapping/buildInfo.mat')

% Generate zip file
    packNGo(buildInfo, 'fileName', 'proc_mapping.zip');

% Delete codeGen folders.
    rmpath(genpath('build'));
    rmpath(genpath('codegen'));
    rmpath(genpath('devel'));
    rmpath(genpath('install'));
    rmpath(genpath('log'));
    rmpath(genpath('msgdef'));
    rmpath(genpath('src'));

    try rmdir('build', 's'); end;
    try rmdir('codegen','s'); end;
    try rmdir('devel','s'); end;
    try rmdir('install','s'); end;
    try rmdir('log','s'); end;
    try rmdir('msgdef','s'); end;
    try rmdir('src','s'); end;   

    disp('Done !');