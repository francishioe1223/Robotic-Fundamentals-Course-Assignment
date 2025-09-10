function [theta1, theta2, theta3, theta4, theta5] = inverse_kinematics(xw, yw, zw, psi, d1, L1, L2, L3, theta1_sol, theta5_sol)
    % Calculate theta1
    theta1 = atan2(yw, xw);

    % Calculate the wrist center position
    xw_3 = xw - (sin(psi)*L3*cos(theta1_sol));
    yw_3 = yw - (sin(psi)*L3*sin(theta1_sol));
    zw_3 = zw + (cos(psi)*L3);

    % Calculate theta3
    rw_3 = sqrt(xw_3^2 + yw_3^2);
    D = -(rw_3^2 + (zw_3 - d1)^2 - L1^2 - L2^2) / (2*L1*L2);
    D = max(min(D, 1), -1); % Clamp D to [-1, 1] to handle numerical issues
    theta3 = atan2(sqrt(1 - D^2), -D);  % Choose the negative elbow configuration

    % Calculate theta2
    beta = atan2(zw_3 - d1, rw_3);
    alpha = atan2(L2*sin(theta3), L1 + L2*cos(theta3));
    theta2 = beta + alpha;

    % Calculate theta4 to achieve desired orientation
    theta4 = psi - theta2 - (-theta3);

    % Assign theta5
    theta5 = theta5_sol;
end