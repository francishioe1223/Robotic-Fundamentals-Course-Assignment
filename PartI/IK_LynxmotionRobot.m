function [t1,t2,t3,t4,t5]  = IK_LynxmotionRobot(xw,yw,zw,psi)
    d1 = 3;
    L1 = 10;
    L2 = 10;
    L3 = 3;
    
    theta1 = atan2(yw, xw);
    theta5 = 0;
    
    xw_3 = xw - (sin(psi)*L3*cos(theta1));
    yw_3 = yw - (sin(psi)*L3*sin(theta1));
    zw_3 = zw + (cos(psi)*L3); 
    
    rw_3 = sqrt( xw_3^2 + yw_3^2 );
    D = -(rw_3^2 + (zw_3 - d1)^2 - L1^2 - L2^2) / (2*L1*L2);
    if D < -1
        D = -1
    elseif D > 1
        D = 1
    end
    theta3 = atan2(sqrt(1 - D^2), -D);  
    
    % Calculate theta2
    beta = atan2(zw_3 - d1, rw_3);
    alpha = atan2(L2*sin(theta3), L1 + L2*cos(theta3));
    theta2 = beta + alpha;
    
    % Calculate theta4 to achieve desired orientation
    theta4 = psi - theta2 - (-theta3); % theta must add (-) due to its oppisite orientation
    
    t1 = round(rad2deg(theta1), 3);
    t2 = round(rad2deg(theta2), 3);
    t3 = round(rad2deg(-theta3), 3); % theta must add (-) due to its oppisite orientation
    t4 = round(rad2deg(theta4), 3);
    t5 = round(rad2deg(theta5), 3);
end
