function [featurePrev, featureCurrent, k] = featurePrep(featurePrev, k, grayPrev, depPrev, grayCurr, depCurr)
% This function prepares the features used for the frame to frame motion
% estimation using Optical Flow KLT
% INPUT: Gray-D images for previous and current frame. All Size [H W] double
%       featurePrev [P 3] P features, each has (x,y,z)
% OUTPUT:featureCurrent [M 3], featurePrev [M 3], M features are tracked
% and used for motion to motion estimation. The first k rows should be the
% features with know depth, and then features with unknown depth




end