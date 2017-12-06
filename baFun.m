function repo = baFun(x)
% x [6 N] vector form
global fID;
%global BAframeNum;

% form H for x 8:x
%
Hs = cell(1,size(fID,2)-1);
for i = 1:size(Hs,2)
   Hs{i} = vect2Htrans( x(:,i) );
end
%}
%{
Hs = cell(1,size(fID,2)-1);
for i = 1:size(Hs,2)
   Hs{i} = vect2Htrans( x(:,1) );
   for j = 2:BAframeNum(i+1)-1
       Hs{i} = vect2Htrans( x(:,j) ) * Hs{i};
   end
end
%}
infnD = [0.01,0   ,0;
            0,0.01,0;
            0,   0,0];
        
nkf = size(fID,2);
nfp = size(fID{1},1);

repo = [];
repo = 0;
% sum for all key frame
for j = 2:nkf
    H = Hs{j-1};
    % sum for all feature point
    for i = 1:nfp
        info = infnD;
        if fID{j}(i,2) == 1% && fID{1}(i,2) == 1
            info = [0.01,0,0;
                    0,0.01,0;
                    0,   0,0.01/(fID{j}(i,5))^2];
%              X_l_telda = [fID{1}(i,3:5)';1];
%              X_j_telda = [fID{j}(i,3:5)';1];  
%              temp = H*X_l_telda - X_j_telda;
%              temp = temp(1:3); % 3x1
%              temp = temp' * info * temp;
%              repo = repo + temp;
        end
             X_l_telda = [fID{1}(i,3:5)';1];
             X_j_telda = [fID{j}(i,3:5)';1];  
             temp = H*X_l_telda - X_j_telda;
             temp = temp(1:3); % 3x1
             temp = temp' * info * temp;
             repo = repo + temp;
%         X_l_telda = [fID{1}(i,3:4)'./fID{1}(i,5);fID{1}(i,5);1];
%         X_j_telda = [fID{j}(i,3:4)'./fID{j}(i,5);fID{j}(i,5);1];
%         temp = H*X_l_telda - X_j_telda;
%         temp = temp(1:3); % 3x1
%         temp = temp' * info * temp;
%         repo = repo + temp;
    end
end

end