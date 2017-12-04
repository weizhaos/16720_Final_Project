function weight = bisquareWeight(residual, C)

    if C == 0
        weight = 1;
    else 
        if abs(residual) <= C
            weight = (1 - (residual/C)^2)^2;
        else
            weight = 0;
        end
    end
end
