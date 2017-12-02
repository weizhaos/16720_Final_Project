function residue = myFun(T, theta, featurePrev, featureCurrent, k)
% INPUT: r [3 1]; T [3 1]; featurePrev [M 3] featureCurrent [M 3]
% OUTPUT: residue [M 1]
%% get R
normm = norm(theta);
thetaSkew = [0    , -theta(3), theta(2) ;
            theta(3) , 0    , -theta(1);
            -theta(2), theta(1) , 0    ;];
R = eye(3) + thetaSkew/normm * sin(normm) + thetaSkew^2/normm^2 * (1-cos(normm));
R_1 = R(1,:);
R_2 = R(2,:);
R_3 = R(3,:);

%% feature with depth
x_bar_k = featureCurrent(1:k,2)./featureCurrent(1:k,3); % [k 1]
y_bar_k = featureCurrent(1:k,1)./featureCurrent(1:k,3); % [k 1]
X_k_1 = featurePrev(1:k,:); % [k 3]

residue1a = diag((repmat(R_1,k,1) - x_bar_k*R_3) * X_k_1') + T(1) - x_bar_k*T(3); % [k 1]
residue1b = diag((repmat(R_2,k,1) - y_bar_k*R_3) * X_k_1') + T(2) - y_bar_k*T(3); % [k 1]

%% feature with unknown depth
x_bar_k = featureCurrent(k+1:size(featureCurrent,1),2); % [M-k 1]
y_bar_k = featureCurrent(k+1:size(featureCurrent,1),1); % [M-k 1]
X_bar_k_1 = featurePrev(k+1:size(featurePrev,1),:); % [k 3]

residue2 = [-y_bar_k*T(3)+T(2), x_bar_k*T(3)-T(1), -x_bar_k*T(2)+y_bar_k*T(1)]*R*X_bar_k_1';
residue2 = diag(residue2); % [M-k 1]

%% stack
residue = [residue1a; residue1b; residue2]; % [2k+(M-k) 1]

end