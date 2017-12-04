function poses = findWorldPoseVect(tmat, gt)
% tmat [6 N]

%% form H_ref
initialPos = [1.3405 0.6266 1.6575 0.6574 0.6126 -0.2949 -0.3248];
angleTemp = quat2rotm(initialPos(4:end));
H_ref = [angleTemp,initialPos(1:3)'; 0,0,0,1];

poses = [0;0;0;1];
H_previous = H_ref;
for i = 2:size(tmat,2)
    temp = vect2Htrans(tmat(:,i)) * H_previous;
    poses(:,i) =  temp * poses(:,1);
    H_previous = temp;
end


% groungTruth [7 N]
% [tx,ty,tz,qw,qx,qy,qz]

%
posesGT = zeros(4,size(gt,2));
for i = 1:size(gt,2)
    angleTemp = quat2rotm(gt(4:end,i)');
    H_tran = [angleTemp,gt(1:3,i); 0,0,0,1];
    posesGT(:,i) =  H_tran * [0;0;0;1];
end