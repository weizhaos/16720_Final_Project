function X = bundleAdjustment()

global BAfeature;
global BAframeNum;
global deltaPosT;
global fID;
global Hs;

frameNum = size(BAfeature,2);
% initialize the registered feature
fID = cell(size(BAfeature));
fID(end) = BAfeature(end);
for i = 1:frameNum-1
   fID{i} = zeros(size(fID{end},1),5); 
end
% register the feature in last keyframe to previous keyframe
for i = 1:size(fID{end},1)
    fid = fID{end}(i,1);
    for j = 1:frameNum-1
        fID{j}(i,:) = BAfeature{j}(BAfeature{j}(:,1)==fid,:);
    end
end
% form H
Hs = cell(size(BAfeature,1),size(BAfeature,2)-1);
for i = 1:size(Hs,2)
   fNum =  BAframeNum(i+1);
   Hs{i} = vect2Htrans(deltaPosT(:,2));
   for j = 3:fNum
       Hs{i} = vect2Htrans(deltaPosT(:,j)) * Hs{i};
   end
end
% start optimization
initX = deltaPosT(:,BAframeNum(1)+1:BAframeNum(end)); % [6 N]
[X, resnorm] = fminsearch(@baFun, initX, ...
                   optimset ('MaxFunEvals', 100000, ...
                         'MaxIter', 1000000, ....
                         'Algorithm', 'levenberg-marquardt'));
fprintf('BA_Error: %f\n',resnorm);
end