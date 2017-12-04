syms xbki ybki xbk1i ybk1i;
syms xk1i yk1i zk1i;
syms tx ty tz;
syms T1 T2 T3;

Xbki = [xbki;ybki;1]; %normalized feature at k
Xbk1i = [xbk1i;ybk1i;1]; %normalized feature at k-1
Xk1i = [xk1i;yk1i;zk1i]; %3d location of feature at k-1

T = [T1;T2;T3]; %translation vector

theta = [tx;ty;tz]; %rotation angles vector

%skew symmetric matrix of theta
theta_cap = [0, -tz, ty;
             tz, 0, -tx;
             -ty, tx, 0];

%theta second norm
tmod = sqrt(tx^2+ty^2+tz^2);

%Rotation matrix expressed in Rodrigues formula
R = eye(3) + theta_cap*sin(tmod)/tmod + (theta_cap^2)*(1-cos(tmod))/tmod^2;

R1 = R(1,:);
R2 = R(2,:);
R3 = R(3,:);

depth_exp1 = (R1 - xbki*R3)*Xk1i + T1 - xbki*T3; %eqn 3
depth_exp2 = (R2 - ybki*R3)*Xk1i + T2 - ybki*T3; %eqn 4

nodepth_exp1 = [-ybki*T3 + T2, xbki*T3 - T1, -xbki*T2 + ybki*T1]*R*Xbk1i; %eqn 6

depth_exp1_Jacobian = [diff(depth_exp1,T1), diff(depth_exp1,T2), diff(depth_exp1,T3), diff(depth_exp1,tx), diff(depth_exp1,ty), diff(depth_exp1,tz)];
depth_exp2_Jacobian = [diff(depth_exp2,T1), diff(depth_exp2,T2), diff(depth_exp2,T3), diff(depth_exp2,tx), diff(depth_exp2,ty), diff(depth_exp2,tz)];

nodepth_exp1_Jacobian = [diff(nodepth_exp1,T1), diff(nodepth_exp1,T2), diff(nodepth_exp1,T3), diff(nodepth_exp1,tx), diff(nodepth_exp1,ty), diff(nodepth_exp1,tz)];
