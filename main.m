 %% Main code structure for 16720 project, Fall 2017
%{
TODO LIST: 
	Find data set and extract 40 images suitable for the task;
	Decide and implement feature detection/decription/matching: Harris, ORB, FAST
    Decide and implement feature tracking: KLT
	Implement Frame to frame motion estimation
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

%% Initialization
fpathRGB = [];
fpathDEP = [];
data = loadData(fpathRGB,fpathDEP);
poses = initialpose(); % [1 6] now, later [N 6]
totalStamp = size(data{1},3);
[featurePrev, k] = featureExtraction([], data{1}(:,:,1), maxFeatNum, distribute);

%% Frame to Frame
% main loop
for i = 2:totalStamp
    % add in data
    grayPrev = data{1}(:,:,i-1);
    depPrev = data{2}(:,:,i-1);
    grayCurr = data{1}(:,:,i);
    depCurr = data{1}(:,:,i);
    % optical Flow the track the feature to current frame
    [featurePrev, featureCurrent, k] = featurePrep(featurePrev, k, grayPrev, depPrev, grayCurr, depCurr);
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
        featurCurrent = featureExtraction(featurCurrent, data{1}(:,:,i), ...
            maxFeatNum-size(featurCurrent,1), distribute);
    end
    featurePrev = featurCurrent;
end
   
%% Save and visualization
% compare to groud truth 

	

