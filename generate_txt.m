function generate_txt(trans, quad)
fpathRgb = 'data/rgbd_dataset_freiburg1_xyz/rgb/';
RGB_imagefiles = dir(strcat(fpathRgb,'/*.png'));
name = zeros(length(RGB_imagefiles)-1,1);
for i = 2 : length(RGB_imagefiles)
    RGB_imagefiles(i).name(end-4:end) = [];
    num = str2num(RGB_imagefiles(i).name);
    name(i-1) = num;
end
format long e 
time_stamp = name;
result = [time_stamp,trans,quad];
result = result';
fileID = fopen('result.txt', 'w');
fprintf(fileID,'%15.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n', result);
fclose(fileID);
end

