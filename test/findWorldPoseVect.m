function [poses,posesGT] = findWorldPoseVect(tmat, gt)
% tmat [6 N]

%% form H_ref
initialPos = [1.3405 0.6266 1.6575 -0.3248 0.6574 0.6126 -0.2949];
angleTemp = quat2rotm(initialPos(4:end));
H_ref = [angleTemp,initialPos(1:3)'; 0,0,0,1];

poses = zeros(4,size(tmat,2));
poses(:,1) =  H_ref * [0;0;0;1];
H_previous = H_ref;
for i = 2:size(tmat,2)
    temp = H_previous * inv(vect2Htrans(tmat(:,i)));
    poses(:,i) =  temp * [0;0;0;1];
    H_previous = temp;
end


% groungTruth [7 N]
% [tx,ty,tz,qw,qx,qy,qz]
posesGT = zeros(4,size(gt,2));
for i = 1:size(gt,2)
    % to use quat2rotm: qw,qx,qy,qz
    angleTemp = quat2rotm(gt(4:end,i)');
    H_tran = [angleTemp,gt(1:3,i); 0,0,0,1];
    posesGT(:,i) =  H_tran * [0;0;0;1];
end