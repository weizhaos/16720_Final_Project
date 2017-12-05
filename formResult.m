function result = formResult(tmat)
% INPUT: tmat [6 N]
% OUTPUT: [N 7]
%% form H_ref
initialPos = [1.3405 0.6266 1.6575 0.6574 0.6126 -0.2949 -0.3248];
angleTemp = quat2rotm(initialPos(4:end));
H_ref = [angleTemp,initialPos(1:3)'; 0,0,0,1];

%% form the first column of the result
result = zeros(size(tmat,2),7);

temp = Htrans2Vect(H_ref);
temp = temp'; % 1 x 6
temp2quat = [temp(4:6)./norm(temp(4:6)) norm(temp(4:6))];
temp = [temp(1:3) axang2quat(temp2quat)];
result(1,:) = temp;

H_previous = H_ref;
for i = 2:size(tmat,2)    
    H_tran = vect2Htrans(tmat(:,i));
    H_new = H_tran * H_previous;   

    temp = Htrans2Vect(H_new);
    temp = temp'; % 1 x 6
    temp2quat = [temp(4:6)./norm(temp(4:6)) norm(temp(4:6))];
    temp = [temp(1:3) axang2quat(temp2quat)];
    result(i,:) = temp;
    H_previous = H_new; 
end


end