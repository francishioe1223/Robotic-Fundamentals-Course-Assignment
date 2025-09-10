function [theta, PP] = inverse_kinematics(Xc, Yc, r, alpha, SA, L, PB)
    % Initialize active joint angles and platform joints
    theta = zeros(3, 1);
    PP = zeros(3, 2);
    c = zeros(3, 1);
    distance = zeros(3, 1);
    d = zeros(3, 1);
    phi = zeros(3, 1);

    phi(1) = alpha + deg2rad(30);
    for i = 2:3
        phi(i) = phi(i-1) + deg2rad(120);
    end

    for i = 1:3
        % Platform Joint Position
        PP(i, 1) = Xc - r * cos(phi(i));
        PP(i, 2) = Yc - r * sin(phi(i));
        
        % Base to Platform Joint Distance
        distance(i) = sqrt((PP(i, 1) - PB(i,1))^2 + (PP(i, 2) - PB(i,2))^2);
        
        % Base to Platform Joint Angle
        c(i) = atan2((PP(i, 2) - PB(i,2)), (PP(i, 1) - PB(i,1)));        
        d(i) = acos((SA^2 - L^2 + distance(i)^2) / (2 * SA * distance(i)));
              
        theta(i, 1) = c(i) + d(i);
        theta(i, 2) = c(i) - d(i);

    end
end