function [Xprev, Xcurrent] = transferToWorldCoord(K, featurePrev, featureCurrent)
% INPUT: featurePrev, featureCurrent [k 3]
% OUTPUT: Xprev, Xcurrent [k 3]
% convert [H,W] to [x,y]
temp = featurePrev(:,1);
featurePrev(:,1) = featurePrev(:,2);
featurePrev(:,2) = temp;
temp = featureCurrent(:,1);
featureCurrent(:,1) = featureCurrent(:,2);
featureCurrent(:,2) = temp;
% times the lambda
% currrent feature does not have depth available
featureCurrent(:,3) = ones(size(featureCurrent,1),1);
featurePrev(:,1) = featurePrev(:,1) .* featurePrev(:,3);
featureCurrent(:,1) = featureCurrent(:,1) .* featureCurrent(:,3);
% conversion
Xprev = K \ featurePrev';
Xcurrent = K \ featureCurrent';
Xprev = Xprev';
Xcurrent = Xcurrent';
end