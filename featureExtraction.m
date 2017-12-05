function [feature, featureID, k] = featureExtraction(image, depth, N, window)
% This function extract a certain number of feature evenly among the image 
% INPUT: image [H W], depth [H W], feature number to extract N
%        distribute [1 2] how should the uniform area form: divide row by
%        the first element, divide column by the second element
% OUTPUT: the extracted features [M 3]
feature_points = detectFASTFeatures(image);
strongest = feature_points.selectStrongest(500);
feature = zeros(0,3);
[~, width] = size(image);
feature_candidate = [strongest.Location,strongest.Metric,ones([size(strongest.Location,1),1])];
k = 0;
for i = 1 : size(strongest.Location,1)
    feature_candidate(i,4) = floor(feature_candidate(i,2)/window(1))*(width/window(2))...
        +(floor(feature_candidate(i,1)/window(2)));  
end
unique_window = unique(feature_candidate(:,4));
for i = 1 : length(unique_window)
    point = feature_candidate(feature_candidate(:,4) == unique_window(i),:,:,:);
    [~,max_ind] = max(point(:,3));
    feature_tmp = [point(max_ind,2),point(max_ind,1)];
    if (depth(feature_tmp(1),feature_tmp(2)) ~= 0)
        k = k + 1;
        feature = [[feature_tmp(1),feature_tmp(2),depth(feature_tmp(1),feature_tmp(2))];feature];
    else
        feature = [feature;[feature_tmp(1),feature_tmp(2),1]];
    end
end
feature = double(feature);
featureID = 1:size(feature,1);
featureID = featureID';
end
%{
figure();
imshow(image)
hold on
for i = 1 : length(feature)
    scatter(feature(i,1),feature(i,2),'b');
end
%}
% feature = zeros(0,3);
% block_height = height/distribute(1);
% block_width = width/distribute(2);
% block_num = distribute(1) * distribute(1);

% image_sets = cell(1, block_num);
% figure;
% imshow(image);
% hold on
% for i = 1 : distribute(1)
%     for j = 1 : distribute(2)
%         image_sets{(i-1)*distribute(2) + j} = image((i - 1)*block_height + 1 : i*block_height ...
%             , (j - 1)*block_width + 1 : j*block_width);
%         feature_points = detectSURFFeatures(image_sets{i});
%         strongest = feature_points.selectStrongest(10);
%         feature_num = size(strongest,1);
%         k = k + feature_num;
%         feature_temp = strongest.Location + repmat([(j - 1)*block_width, (i - 1)*block_height], [feature_num,1]);
%         feature_temp = [feature_temp,repmat(0,[feature_num,1])];
%         feature = [feature;feature_temp];
%     end
% end
% for i = 1 : length(feature)
%     scatter(feature(i,1),feature(i,2));
% end
%end