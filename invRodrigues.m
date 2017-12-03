function r = invRodrigues(R)
% invRodrigues
% Input:
%   R: 3x3 rotation matrix
% Output:
%   r: 3x1 vector
% refered: https://www.cs.duke.edu/courses/fall13/compsci527/notes/rodrigues.pdf
%% Initialization
A = (R - R')/2;
rho = [A(3,2), A(1,3), A(2,1) ]';
s = norm(rho);
c = (trace(R) - 1)/2;

r = [0;0;0];

%% To get r in different cases
if abs(s) < eps && abs(c-1) < eps
    r = [0;0;0];
elseif abs(s) < eps && abs(c+1) < eps
    temp = R + eye(3);
    [~,index] = max(diag(temp));
    v = temp(:,index);
    u = v/norm(v);
    r = pi.*u;
    if abs(norm(r)-pi) < eps && ( (abs(r(1))<eps && abs(r(2))<eps && r(3)<0) ...
            || (abs(r(1))<eps && r(2)<0) || (r(1)<0) )
        r = -r;
    end
else
    theta = 0;
    if c > 0
        theta = atan(s/c);
    elseif c < 0
        theta = pi + atan(s/c);
    elseif c == 0 && s > 0
        theta = pi/2;
    elseif c == 0 && s < 0
        theta = -pi/2;
    end
    u = rho/s;
    r = theta*u;
    if abs(norm(r)-pi) < eps && ( (abs(r(1))<eps && abs(r(2))<eps && r(3)<0) ...
            || (abs(r(1))<eps && r(2)<0) || (r(1)<0) )
        r = -r;
    end
end
 
end
