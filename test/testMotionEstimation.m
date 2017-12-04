%{
directory = '/Users/srini/VirtualBox VMs/Ubuntu shared/motionEstimation/dataset_20151209/bag3/';
features_2d_prefix = 'features/feature';
features_3d_prefix = 'features_with_depth/feature_with_depth';
frame_prefix = 'images/image';
save_prefix = 'processed/';

% gt_file = 'bag1_imar2camera.mat';
% load(gt_file);
vopose_bag = load([directory,'bag3_position_bagplay0_05.csv']);
vopose_T = vopose_bag(:,4:6)';
vopose_t = quat2eul([vopose_bag(:,10),vopose_bag(:,7:9)])';
%}

startFrame = 1;
endFrame = 798;

%using vo_pose to initialize
T__k_1__0 = vect2Htrans([vopose_T(:,startFrame+1);vopose_t(:,startFrame+1)])^(-1);
T__k__0 = vect2Htrans([vopose_T(:,startFrame+2);vopose_t(:,startFrame+2)])^(-1);

vT__k__k_1 = zeros(6,1) + 1e-15;

%for debugging
Tmat = zeros(3,endFrame-startFrame);
tmat = zeros(3,endFrame-startFrame);
Emat = zeros(1, endFrame-startFrame);
ProjErrMat = zeros(1,endFrame-startFrame);
gtProjErrMat = zeros(1,endFrame-startFrame);
state = zeros(1,endFrame-startFrame);

%feeding in vo pose as ground truth
gt_T = vopose_T;
gt_t = vopose_t;

global features;
global C_nodepth;
global C_depth_x;
global C_depth_y;

C_nodepth = 0.001;%0.5;%0.7;%0.1;
C_depth_x = 1.2;%2.2;%3;%1.2;%0.3;
C_depth_y = 1.2;

for i=startFrame:endFrame

    fprintf('Frame: %d\n', i);
    
    if i ~= startFrame
        f_w_d_k_1 = f_w_d_k;
        f_wo_d_k_1 = f_wo_d_k;
    end
    
    %read the features_3d file for k
    f_w_d_k = load([directory, features_3d_prefix, num2str(sprintf('%05d',i)),'.txt']);

    %read the features_2d file for k
    f_wo_d_k = load([directory, features_2d_prefix, num2str(sprintf('%05d',i)), '.txt']);
    %pixel coordinates to normalized coordinates
    f_wo_d_k(:,2) = (f_wo_d_k(:,2) - cxl)./focal_length; 
    f_wo_d_k(:,3) = (f_wo_d_k(:,3) - cy)./focal_length;

    if i == startFrame
        continue;
    end
    
    %move the 3d points to k-1 frame
    pts3d = f_w_d_k_1(:,2:4);
    f_w_d_k_1(:,2:4) = movePoints(pts3d, T__k_1__0);
    
    %% co-relate features in features with and without depth in k-1 frame and
    %features without depth in k frame (f_w_d_k_1, f_wo_d_k_1 and f_wo_d_k)
    
    %create the feature vector
    features = zeros(size(f_wo_d_k_1,1), 9);
    featureCounter = 1;
    
    for j=1:size(f_wo_d_k_1,1)
        currFid = f_wo_d_k_1(j,1);
            
        features(featureCounter,1) = currFid;
        features(featureCounter,2:3) = f_wo_d_k_1(j,2:3);
        
        %check if we have the depth
        depth_index = find(f_w_d_k_1(:,1) == currFid);
        if ~isempty(depth_index)
            features(featureCounter,4) = 1;
            features(featureCounter,5:7) = f_w_d_k_1(depth_index, 2:4);
        else
        %clear this as it could have been filled in the previous iteration
            features(featureCounter,4:7) = [0,0,0,0];
        end
        
        %check if we have the feature in kth frame
        k_index = find(f_wo_d_k(:,1) == currFid);
        if ~isempty(k_index)
            features(featureCounter,8:9) = f_wo_d_k(k_index, 2:3);
        end
        
        if isempty(k_index)
          %there is no use of a feature which is not tracked in the k th frame
          %lets allow the current row to be overwritten
        else
            featureCounter = featureCounter + 1;
        end
    end
    
    features = features(1:featureCounter-1,:);
    %save(['features-',num2str(i-1),'_',num2str(i),'.mat'], 'features');
    
    %% now feed it to the motion estimation step

    reprojectFn = @(x) reprojectionFn(x);
    %[T,t,finishState] = LevenbergMarquardt(reprojectFn, T, t);


    options = optimset('Jacobian','on');
    %using build-in matlab function

    %x = invertTransform([T;t]);
    
    [vT__k__k_1, resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin('reprojectionFn',vT__k__k_1, [], [], options);
    fprintf('Error: %f\n',resnorm);
    
    frames{1} = [directory, frame_prefix, num2str(sprintf('%05d',i-1)),'.png'];
    frames{2} = [directory, frame_prefix, num2str(sprintf('%05d',i)),'.png'];

    save_frames{1} = [directory,save_prefix,'k_1/image_k_1_',num2str(sprintf('%05d',i-1)),'_',num2str(sprintf('%05d',i)),'.jpg'];
    %save_frames{1} = [directory,save_prefix,num2str(sprintf('%05d',i-1)),'_',num2str(sprintf('%05d',i)),'.jpg'];
    save_frames{2} = [directory,save_prefix,'k/image_k_',num2str(sprintf('%05d',i-1)),'_',num2str(sprintf('%05d',i)),'.jpg'];
    [projErr, gtProjErr] = displayReprojection(frames, save_frames, features, vT__k__k_1, [gt_T(:,i);gt_t(:,i)],[gt_T(:,i+1);gt_t(:,i+1)]);
    
    T__k__k_1 = vect2Htrans(vT__k__k_1);
    T__0__k = T__k_1__0^(-1) * T__k__k_1^(-1);
    vT__0__k = Htrans2Vect(T__0__k);

    Tmat(:,i-startFrame+1) = vT__0__k(1:3);
    tmat(:,i-startFrame+1) = vT__0__k(4:6);
    Emat(:,i-startFrame+1) = resnorm;
    ProjErrMat(:,i-startFrame+1) = projErr;
    gtProjErrMat(:,i-startFrame+1) = gtProjErr;
    
    %state(:,i-startFrame+1) = finishState;
    T__0__k_1 = T__0__k;

end
            
save([directory,'bag3_pose_',datestr(clock,'yyyy-mm-dd-HH_MM'),...
    '.mat'],'Tmat','tmat','Emat','ProjErrMat','gtProjErrMat','C_nodepth','C_depth_x','C_depth_y');

