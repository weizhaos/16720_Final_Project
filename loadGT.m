function loadGT()
%
gt = fileread('../data/rgbd_dataset_freiburg1_xyz/groundtruth.txt');
t1 = gt(101:166);
% compare to groud truth 
file = fileread('../data/rgbd_dataset_freiburg1_xyz/groundtruth.txt');
k = strfind(file, '1305');
tx = zeros(3000,1);
ty = zeros(3000,1);
tz = zeros(3000,1);
qx = zeros(3000,1);
qy = zeros(3000,1);
qz = zeros(3000,1);
qw = zeros(3000,1);
for i = 352 : 3000
    tx(i) = str2double(strcat(file(k(i)+16),file(k(i)+17),file(k(i)+18),strcat(file(k(i)+19),file(k(i)+20),file(k(i)+21))));
    ty(i) = str2double(strcat(file(k(i)+23),file(k(i)+24),file(k(i)+25),strcat(file(k(i)+26),file(k(i)+27),file(k(i)+28))));
    tz(i) = str2double(strcat(file(k(i)+30),file(k(i)+31),file(k(i)+32),strcat(file(k(i)+33),file(k(i)+34),file(k(i)+35))));
    qx(i) = str2double(strcat(file(k(i)+37),file(k(i)+38),file(k(i)+39),strcat(file(k(i)+40),file(k(i)+41),file(k(i)+42))));
    qy(i) = str2double(strcat(file(k(i)+44),file(k(i)+45),file(k(i)+46),strcat(file(k(i)+47),file(k(i)+48),file(k(i)+49))));
    qz(i) = str2double(strcat(file(k(i)+51),file(k(i)+52),file(k(i)+53),strcat(file(k(i)+54),file(k(i)+55),file(k(i)+56))));
    qw(i) = str2double(strcat(file(k(i)+58),file(k(i)+59),file(k(i)+60),strcat(file(k(i)+61),file(k(i)+62),file(k(i)+63))));
end
tx=tx(tx~=0);
ty=ty(ty~=0);
tz=tz(tz~=0);
qx=qx(qx~=0);
qy=qy(qy~=0);
qz=qz(qz~=0);
qw=qw(qw~=0);
save('ground_truth.mat','tx','ty','tz','qx','qy','qz','qw');
%}
end