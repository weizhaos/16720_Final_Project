function addtoBA(feat,featureID,k,frameNum)

global BAfeature;
global BAframeNum;

temp = zeros(size(feat,1),4);
temp(:,1) = featureID;
temp(:,2) = [ones(k,1);zeros(size(feat,1)-k,1)];
temp(:,3:4) = feat(:,1:2)./feat(:,3);
temp(:,5) = feat(:,3);

BAfeature = [BAfeature,temp];
BAframeNum = [BAframeNum,frameNum];
end