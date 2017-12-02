 %% Main code structure for 16720 project, Fall 2017
%{
TODO LIST: 
	debug Frame to frame motion estimation
	Implement BA
	Save/Compare results from FTF, BA, groudtruth
%}
%% global parameters
maxFeatNum = 100; % The maximum number of features to keep in one frame
minFeatNum = 50; % The minimum number of features to keep in one frame
distribute = [32,32]; % How should the uniform grid form for feature extraction
BAGap = 0;% 5 image-to-save for BA gap
BADo = 0;% 40 how many images to do one BA
%load intrinsicROSDefault.mat

%% Initialization
% fpathRGB = '../data/rgbd_dataset_freiburg1_xyz/rgb/';
% fpathDEP = '../data/rgbd_dataset_freiburg1_xyz/depth/';
% fpathFLO = '../data/rgbd_dataset_freiburg1_xyz/rgb/';
%data = loadData(fpathRGB,fpathDEP);
%flow = loadFlow(fpathFLO);
%load ../data/flow.mat
%load ../data/data.mat
poses = initialPose(); % [1 6] now, later [N 6]
totalStamp = size(data{1},3);
[featurePrev, k] = featureExtraction(data{1}(:,:,1), data{2}(:,:,1), maxFeatNum, distribute);

%% Frame to Frame
% main loop
for i = 2:totalStamp
    % add in data
    grayPrev = [];% data{1}(:,:,i-1);
    depPrev = [];% data{2}(:,:,i-1);
    grayCurr = [];% data{1}(:,:,i);
    depCurr = data{1}(:,:,i);
    flowmap = flow{i-1};
    % optical Flow the track the feature to current frame
    [featurePrev, featureCurrent, k] = featurePrep(featurePrev, k, flowmap, ...
        grayPrev, depPrev, grayCurr, depCurr);
    % solve frame to frame motion estimation
    poseTemp = motionEstimation(featurePrev, featureCurrent, k, poses(end,:));
    % add in poses
    poses = [poses; poseTemp];
    % BA
    if mod(i,BAGap) == 0
    end
    if mod(i,BADo) == 0
        bundleAdjustment();
    end
    % update previous feature vector
    if size(featureCurrent,1) <= minFeatNum
        % or directly re-extract all features, which should be better
        [featurCurrent, k] = featureExtraction(data{1}(:,:,i), ...
            data{2}(:,:,i), maxFeatNum, distribute);
    end
    featurePrev = featurCurrent;
    depthPrev = depthCurrent;
end
   
%% Save and visualization
	

