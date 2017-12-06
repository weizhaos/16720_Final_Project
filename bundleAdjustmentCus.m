function X = bundleAdjustmentCus()

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
% form H, initX
Hs = cell(1,size(BAfeature,2)-1);
initX = zeros(6,size(BAfeature,2)-1);
finit =  BAframeNum(1)+1;
for i = 1:size(Hs,2)
   fNum = BAframeNum(i+1);
   Hs{i} = vect2Htrans(deltaPosT(:,finit));
   for j = finit+1:fNum
       Hs{i} = vect2Htrans(deltaPosT(:,j)) * Hs{i};
   end
   initX(:,i) = Htrans2Vect(Hs{i}); 
end

%initX = deltaPosT(:,finit:fNum);
% start optimization
[X, resnorm] = fminsearch(@baFun, initX, ...
                   optimset ('MaxFunEvals', 10000, ...
                         'MaxIter', 10000, ....
                         'Algorithm', 'levenberg-marquardt'));
% options.Algorithm = 'levenberg-marquardt';
% [X, resnorm] = lsqnonlin(@baFun,initX,[],[],options);

fprintf('BA_Error: %f\n',resnorm);

end
