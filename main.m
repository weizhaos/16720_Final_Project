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
    depCurr = data{1}(:,:,i);
    flowmap = flow{i-1};
    % optical Flow to track the feature to current frame
    [featurePrev, featureCurrent, k] = featurePrep(featurePrev, k, flowmap, ...
        grayPrev, depPrev, grayCurr, depCurr);
    % transfer to 3D world coordinates
    [Xprev, Xcurrent] = transferToWorldCoord(K, featurePrev, featureCurrent);
    % solve frame to frame motion estimation
    deltaPos = motionEstimation(Xprev, Xcurrent, k, initialPose());
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
gt = fileread('../data/rgbd_dataset_freiburg1_xyz/groundtruth.txt');
t1 = gt(101:166);
% compare to groud truth 
file = fileread('../data/rgbd_dataset_freiburg1_xyz/groundtruth.txt');
k = strfind(file, '1305');
tx = zeros(3000,1);
ty = zeros(3000,1);
tz = zeros(3000,1);
qx = zeros(3000,1);
qy = zeros(3000,1);
qz = zeros(3000,1);
qw = zeros(3000,1);
for i = 352 : 3000
    tx(i) = str2double(strcat(file(k(i)+16),file(k(i)+17),file(k(i)+18),strcat(file(k(i)+19),file(k(i)+20),file(k(i)+21))));
    ty(i) = str2double(strcat(file(k(i)+23),file(k(i)+24),file(k(i)+25),strcat(file(k(i)+26),file(k(i)+27),file(k(i)+28))));
    tz(i) = str2double(strcat(file(k(i)+30),file(k(i)+31),file(k(i)+32),strcat(file(k(i)+33),file(k(i)+34),file(k(i)+35))));
    qx(i) = str2double(strcat(file(k(i)+37),file(k(i)+38),file(k(i)+39),strcat(file(k(i)+40),file(k(i)+41),file(k(i)+42))));
    qy(i) = str2double(strcat(file(k(i)+44),file(k(i)+45),file(k(i)+46),strcat(file(k(i)+47),file(k(i)+48),file(k(i)+49))));
    qz(i) = str2double(strcat(file(k(i)+51),file(k(i)+52),file(k(i)+53),strcat(file(k(i)+54),file(k(i)+55),file(k(i)+56))));
    qw(i) = str2double(strcat(file(k(i)+58),file(k(i)+59),file(k(i)+60),strcat(file(k(i)+61),file(k(i)+62),file(k(i)+63))));
end
% plot groundtruth
poseGT = [0,0,0];
for i = 2:3000
   poseGT = [poseGT;[poseGT(i-1,1)+tx(i),poseGT(i-1,2)+ty(i),poseGT(i-1,3)+tz(i)]]; 
end
plot3(tx(i),ty(i),tz(i));
	

