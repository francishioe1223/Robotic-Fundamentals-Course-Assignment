function [xw,yw,zw,psi] = FK_LynxmotionRobot(theta1,theta2,theta3,theta4,theta5)
    d1 = 3;
    L1 = 10;
    L2 = 10;
    L3 = 3;
    
    % convert to rad
    theta1 = deg2rad(theta1);
    theta2 = deg2rad(theta2);
    theta3 = deg2rad(theta3);
    theta4 = deg2rad(theta4);
    theta5 = deg2rad(theta5);

    T01 = DistalDH([0 pi/2 d1 theta1]);
    T12 = DistalDH([L1 0 0 theta2]);
    T23 = DistalDH([L2 0 0 theta3]);
    T34 = DistalDH([0 pi/2 0 theta4]);
    T45 = DistalDH([0 0 L3 theta5]);
    T05 = T01 * T12 * T23 * T34 * T45;
    

    xw = T05(1,4);
    yw = T05(2,4);
    zw = T05(3,4);
    psi = theta2+theta3+theta4;
end