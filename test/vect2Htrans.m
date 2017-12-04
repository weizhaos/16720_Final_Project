function H = vect2Htrans(x)
    H = [theta2rot(x(4:6)'),x(1:3); 0,0,0,1];    
end
