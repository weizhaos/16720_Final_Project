function R = rodrigues(r)
% rodrigues:

% Input:
%   r - 3x1 vector
% Output:
%   R - 3x3 rotation matrix
R = [];
%% hyperparameter
%epsilon = 1e-4;

%% get the magnitude and angle direction
theta = norm(r);
if theta == 0
    R = eye(3);
else
    normm = norm(r);
    thetaSkew = [0    , -r(3), r(2) ;
                r(3) , 0    , -r(1);
                -r(2), r(1) , 0    ;];
    %% recover R
    %if theta > epsilon
        R = eye(3) + thetaSkew/normm * sin(normm) + thetaSkew^2/normm^2 * (1-cos(normm));
    %else
    %    R = eye(3) + thetaSkew + 1/2 * thetaSkew^2;
        %R = eye(3)*cos(theta) + (1-cos(theta)).*u*u' + ux*sin(theta);
    %end
end

end
