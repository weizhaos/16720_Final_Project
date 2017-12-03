function deltaPos = motionEstimation(Xprev, Xcurrent, numD, deltaPosInit)
% This function implements algorithm one
% INPUT: featurePrev [M 3] 
%        featureCurrent [M 3] The first k rows of them are features with
%        known depth
%        posOrin [1 6]: the initial pose to optimize from
% OUTPUT: pose is the optimized frame to frame estimated pose
%% hyperparamers
epsiT = 5e-3; % LM converge check
epsiTheta = 5e-3; % LM converge check
iter = 100; % LM max itertaion
lambda = 0.01; % LM damping value
DROP = 0.1 ; % LM damping factor
BOOST = 1.5; % LM damping factor
lastSOS = Inf; % Sum of Square
lastddeltaPose = 0; % LM delta vector

%% inner solver
options.Algorithm = 'levenberg-marquardt';
options.MaxFunctionEvaluations = 10000;
fun = @(deltPos) myFun(deltPos, Xprev, Xcurrent, numD);
[deltaPos,resnorm] = lsqnonlin(fun,deltaPosInit,[],[],options);
resnorm
%}
% minPose = fminunc(@myFun, pose, ...
%            optimset ('MaxFunEvals', 10000, ...
%                      'MaxIter', 100000, ....
%                      'Algorithm', 'levenberg-marquardt'), ...%, ...
%                      featurePrev, featureCurrent, numD);
%{
%% calculate Jacobian
syms dp1 dp2 dp3 dp4 dp5 dp6
residue = myFun([dp1 dp2 dp3 dp4 dp5 dp6], Xprev, Xcurrent, numD);
jacoSYM = jacobian(residue,[dp1 dp2 dp3 dp4 dp5 dp6]);

%% stack for optimization
deltaPos = deltaPosInit;
for k = 1:iter
    % assign bisquare weight [2*numD+(M-numD) 1]
    residue = myFun(deltaPos, Xprev, Xcurrent, numD); 
    MAD = median(abs(residue-median(residue)));
    temp = 1-( residue./(6*MAD) ).^2;
    temp(temp<0) = 0;
    weights = temp.^2;
    residue = weights.*residue;
    % obtain numerical jacobian
    dp1 = deltaPos(1); dp2 = deltaPos(2); dp3 = deltaPos(3);
    dp4 = deltaPos(4); dp5 = deltaPos(5); dp6 = deltaPos(6);
    jacoNUM = double(subs(jacoSYM)); % [2*numD+(M-numD) 6]
    % update damping factor
    SOS = sum(residue.^2);
    if lastSOS ~= Inf
        if SOS < lastSOS
            lambda = DROP*lambda;
            lastdeltaPose = 0;
        else
            lambda = BOOST*lambda;
            lastdeltaPose = DdeltaPose';
        end
    end
    lastSOS = SOS;
    % DO LM
    DdeltaPose = (jacoNUM'*jacoNUM + lambda*diag(jacoNUM'*jacoNUM)) \ (jacoNUM'*residue);
    % update r T
    deltaPos = deltaPos + lastddeltaPose - DdeltaPose';
    % converged
    if norm(DdeltaPose(1:3))<epsiT && norm(DdeltaPose(4:6))<epsiTheta
        break;
    end
end
%}
end