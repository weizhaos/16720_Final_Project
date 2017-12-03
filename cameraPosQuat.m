function [pos] = cameraPosQuat(initialPos,delta)
% assume initialPos [1,3]
% assume delta [N,7]
% pos return type: [N+1 3]
    delta_t = delta(:,1:3);
    delta_r = delta(:,4:7);
    pos = zero(size(delta,1)+1,3);
    pos(1,:) = initialPos;
    for i = 1:size(delta,1)
        pos(i+1,:) = quat2rotm(delta_r(i,:))*pos(i,:) + delta_t(i,:);
    end
end