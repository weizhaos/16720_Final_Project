function [featurePrev, featureCurrent, k] = featurePrep(featurePrev, k, flowmap, grayPrev, depPrev, grayCurr, depCurr)
% This function prepares the features used for the frame to frame motion
% estimation using Optical Flow KLT
% INPUT: Gray-D images for previous and current frame. All Size [H W] double
%       featurePrev [P 3] P features, each has (x,y,z)
% OUTPUT:featureCurrent [M 3], featurePrev [M 3], M features are tracked
% and used for motion to motion estimation. The first k rows should be the
% features with know depth, and then features with unknown depth
imgsize = size(flowmap); % [height, width, 2]
nFeature = size(featurePrev,1);
i = 1;
while(i<=k)
    pos = featurePrev(i,1:2);
    flow = flowmap(floor(pos(1)),floor(pos(2)),:);
    new_pos = floor(pos + [flow(2) flow(1)]);
    if new_pos(1) <= 0 || new_pos(1) > imgsize(1) || new_pos(2) <= 0 || new_pos(2) > imgsize(2)
        % remove the feature
        featurePrev(i,:) = [];
        k = k - 1;
        nFeature = nFeature - 1;
    else
        featureCurrent(i,:) = [new_pos(1) new_pos(2) depCurr(new_pos(1),new_pos(2))];
        i = i+1;
    end
end
% without depth
i = k+1;
while(i<=nFeature)
    pos = featurePrev(i,1:2);
    flow = flowmap(floor(pos(1)),floor(pos(2)),:);
    new_pos = floor(pos + [flow(2) flow(1)]);
    if new_pos(1) <= 0 || new_pos(1) > imgsize(1) || new_pos(2) <= 0 || new_pos(2) > imgsize(2)
        % remove the feature
        featurePrev(i,:) = [];
        nFeature = nFeature - 1;
    else
        i = i+1;
        featureCurrent(i,:) = [new_pos(1) new_pos(2) 1];
    end
end

end
