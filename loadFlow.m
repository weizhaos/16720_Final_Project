function flow = loadFlow(fpathFLO)
% This function extracts rhe Optical Flow result
FLO_imagefiles = dir(strcat(fpathFLO,'/*.flo'));
FLOW_file_num = length(FLO_imagefiles);

flow = cell(1,FLOW_file_num);

for i = 1:FLOW_file_num
    filename = strcat(fpathFLO, FLO_imagefiles(i).name);
    flow{i} = readFlowFile(filename);
end
% save('../data/flow.mat','flow','-v7.3')
end