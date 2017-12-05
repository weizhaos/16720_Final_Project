function x = Htrans2Vect(H)
    
    x = zeros(6,1);
    x(1:3) = H(1:3,4);
    x(4:6) = rot2theta(H(1:3,1:3))';

end
