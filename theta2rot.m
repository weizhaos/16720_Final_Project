function R = theta2rot(theta)

    tx = theta(1);
    ty = theta(2);
    tz = theta(3);
    
    %skew symmetric matrix of theta
    theta_cap = [0, -tz, ty;
                 tz, 0, -tx;
                 -ty, tx, 0];

    %theta second norm
    tmod = sqrt(tx^2+ty^2+tz^2);

    R = eye(3) + theta_cap*sin(tmod)/tmod + (theta_cap^2)*(1-cos(tmod))/tmod^2;


end
