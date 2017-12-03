function pose = motionEstimation(featurePrev, featureCurrent, numD, poseOrin)
% This function implements algorithm one
% INPUT: featurePrev [M 3] 
%        featureCurrent [M 3] The first k rows of them are features with
%        known depth
%        posOrin [1 6]: the initial pose to optimize from
% OUTPUT: pose is the optimized frame to frame estimated pose
%% hyperparamers
epsiT = 3e-4; % LM converge check
epsiTheta = 5e-3; % LM converge check
iter = 100; % LM max itertaion
lambda = 0.01; % LM damping value
DROP = 0.1 ; % LM damping factor
BOOST = 1.5; % LM damping factor
lastSOS = Inf; % Sum of Square
lastdeltaPose = 0; % LM delta vector
pose = poseOrin; % initial pose % constant velocity model ???

%% conver [H,W] to [x,y]
temp = featurePrev(:,1);
featurePrev(:,1) = featurePrev(:,2);
featurePrev(:,2) = temp;

temp = featureCurrent(:,1);
featureCurrent(:,1) = featureCurrent(:,2);
featureCurrent(:,2) = temp;
%% calculate Jacobian
syms T1 T2 T3 theta1 theta2 theta3 
residue = myFun([T1 T2 T3], [theta1 theta2 theta3], featurePrev, featureCurrent, numD);
jacoSYM = jacobian(residue,[T1 T2 T3 theta1 theta2 theta3]);

%% stack for optimization
for k = 1:iter
    % get angle, transition
    T = pose(1:3);
    theta = pose(4:6);
    % assign bisquare weight [2*numD+(M-numD) 1]
    residue = myFun(T, theta, featurePrev, featureCurrent, numD); 
    MAD = median(abs(residue-median(residue)));
    temp = 1-( residue./(6*MAD) ).^2;
    temp(temp<0) = 0;
    weights = temp.^2;
    residue = weights.*residue;
    % obtain numerical jacobian
    theta1 = theta(1); theta2 = theta(2); theta3 = theta(3);
    T1 = T(1); T2 = T(2); T3 = T(3);
    jacoNUM = double(subs(jacoSYM)); % [2*numD+(M-numD) 6]
    % update damping factor
    SOS = sum(residue.^2);
    if lastSOS ~= Inf
        if SOS < lastSOS
            lambda = DROP*lambda;
            lastdeltaPose = 0;
        else
            lambda = BOOST*lambda;
            lastdeltaPose = deltaPose';
        end
    end
    lastSOS = SOS;
    % DO LM
    deltaPose = inv(jacoNUM'*jacoNUM + lambda*diag(jacoNUM'*jacoNUM))*jacoNUM'*residue;
    % update r T
    pose = pose + lastdeltaPose - deltaPose';
    % converged
    if norm(deltaPose(1:3))<epsiT && norm(deltaPose(4:6))<epsiTheta
        break;
    end
end

end