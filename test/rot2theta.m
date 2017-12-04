function t = rot2theta(R)

    axang = rotm2axang(R);
    axnorm = norm(axang(1:3));
    axang(1:3) = axang(1:3)./axnorm;
    t = (axang(1:3).*axang(4))';
    if sum(t) == 0
        %if t is zero set it to some really small value
        t = t + 1e-15;
    end

end