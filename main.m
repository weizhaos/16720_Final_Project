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
load intrinsicROSDefault.mat
K = [fx, 0 , cx;
     0 , fy, cy;
     0 , 0 ,  1;];
 
%% Initialization
% fpathRGB = '../data/rgbd_dataset_freiburg1_xyz/rgb/';
% fpathDEP = '../data/rgbd_dataset_freiburg1_xyz/depth/';
% fpathFLO = '../data/rgbd_dataset_freiburg1_xyz/rgb/';
%data = loadData(fpathRGB,fpathDEP);
%flow = loadFlow(fpathFLO);
%load ../data/flow.mat
%load ../data/data.mat
deltaPos = initialPose(); % [1 6] now, later [N 6]
totalStamp = size(data{1},3);
[featurePrev, k] = featureExtraction(data{1}(:,:,1), data{2}(:,:,1), maxFeatNum, distribute);

%% Frame to Frame
% main loop
deltaPosT = [];
for i = 2:totalStamp
    % add in data
    grayPrev = [];% data{1}(:,:,i-1);
    depPrev = [];% data{2}(:,:,i-1);
    grayCurr = [];% data{1}(:,:,i);
    depCurr = data{2}(:,:,i);
    flowmap = flow{i-1};
    % optical Flow to track the feature to current frame
    [featurePrev, featureCurrent, k] = featurePrep(featurePrev, k, flowmap, ...
        grayPrev, depPrev, grayCurr, depCurr);
    % transfer to 3D world coordinates
    [Xprev, Xcurrent] = transferToWorldCoord(K, featurePrev, featureCurrent);
    % solve frame to frame motion estimation
    deltaPos = motionEstimation(Xprev, Xcurrent, k, deltaPos);
    % add in poses
    deltaPosT = [deltaPosT; deltaPos];
    % poses = [poses; poses(end,:)+deltaPos];
    % BA
    if mod(i,BAGap) == 0
    end
    if mod(i,BADo) == 0
        bundleAdjustment();
    end
    % update previous feature vector
    if size(featureCurrent,1) <= minFeatNum
        % or directly re-extract all features, which should be better
        [featureCurrent, k] = featureExtraction(data{1}(:,:,i), ...
            data{2}(:,:,i), maxFeatNum, distribute);
    end
    featurePrev = featureCurrent;
    depPrev = depCurr;
end
   
%% Save and visualization
%loadGT();
load('ground_truth.mat');
deltaPosGT = [tx,ty,tz,qw,qx,qy,qz];
poseGT = cameraPosQuat([0,0,0],deltaPosGT);
% plot groundtruth
plot3(poseGT(:,1),poseGT(:,2),poseGT(:,3),'g.');
%}	
% deal with estimated result
posET = cameraPos([0,0,0],deltaPosT);
plot3(posET(:,1),posET(:,2),posET(:,3));