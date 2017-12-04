function data = loadData(fpathRGB,fpathDEP)
% This function get input data prepared. Depth should be associated
% INPUT: the file path of RGB image and Depth image
% OUTPUT: data is a [1 2] cell, the first element is gray image [H W T] where H and W is
%       the height and width, the second element is depth image [H W T]. T is the total timestamp. 
%       This data should be aligned in terms of the timestamp. 
%       That is, data{1}(:,:,t) and data{2}(:,:,t) should correspond to each other
data = cell(1,2);
%read images
RGB_imagefiles = dir(strcat(fpathRGB,'/*.png'));

%DEP_imgefiles = dir(strcat(fpathDEP,'/*.png'));
DEP_imgefiles = dir(strcat(fpathDEP,'/*.depth'));


RGB_file_num = length(RGB_imagefiles);
DEP_file_num = length(DEP_imgefiles);
RGB_file_num = 100;
DEP_file_num = 100;
assert(RGB_file_num == DEP_file_num);

[img_height, imwidth, ~] = size(imread(strcat(fpathRGB,'/', RGB_imagefiles(1).name)));
data{1} = zeros(img_height,imwidth,RGB_file_num);
data{2} = zeros(img_height,imwidth,DEP_file_num);
for i = 1 : RGB_file_num
    data{1}(:,:,i) = im2double(rgb2gray(imread(strcat(fpathRGB,'/', RGB_imagefiles(i).name))));
end
for i = 1 : DEP_file_num
    temp = dlmread(strcat(fpathDEP,'/', DEP_imgefiles(i).name));
    data{2}(:,:,i) = reshape(temp,[img_height, imwidth]);
end
