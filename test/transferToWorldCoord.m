function [Xprev, Xcurrent] = transferToWorldCoord(featurePrev, featureCurrent)
% INPUT: featurePrev, featureCurrent [k 3]
% OUTPUT: Xprev, Xcurrent [k 3]
%
fx = 525.0;  % focal length x
fy = 525.0;  % focal length y
cx = 319.5;  % optical center x
cy = 239.5;  % optical center y
K = [fx,0,cx;0,fy,cy;0,0,1];

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
featurePrev(:,2) = featurePrev(:,2) .* featurePrev(:,3);
%featureCurrent(:,1) = featureCurrent(:,1) .* featureCurrent(:,3);
%featureCurrent(:,2) = featureCurrent(:,2) .* featureCurrent(:,3);
% conversion
Xprev = K \ featurePrev';
Xcurrent = K \ featureCurrent';
Xprev = Xprev';
Xcurrent = Xcurrent';
%}
%{
fx = 525.0;  % focal length x
fy = 525.0;  % focal length y
cx = 319.5;  % optical center x
cy = 239.5;  % optical center y

Xprev = zeros(size(featurePrev));
Xcurrent = zeros(size(featureCurrent));

for i = 1:size(featurePrev,1)
    v = featurePrev(i,1);
    u = featurePrev(i,2);
    Z = featurePrev(i,3);
    Xprev(i,2) = (u - cx) * Z / fx;
    Xprev(i,1) = (v - cy) * Z / fy;
    Xprev(i,3) = Z;
end

for i = 1:size(featureCurrent,1)
    v = featureCurrent(i,1);
    u = featureCurrent(i,2);
    Z = 1;
    Xcurrent(i,2) = (u - cx) * Z / fx;
    Xcurrent(i,1) = (v - cy) * Z / fy;
    Xcurrent(i,3) = Z;
end
%}
end