function joint_positions = Calc_JointPos(theta)
    d1 = 3;
    L1 = 10;
    L2 = 10;
    L3 = 3;

    theta_rad = deg2rad(theta);
    theta1 = theta_rad(1);
    theta2 = theta_rad(2);
    theta3 = theta_rad(3);
    theta4 = theta_rad(4);
    theta5 = theta_rad(5);

    T01 = DistalDH([0, pi/2, d1, theta1]);
    T12 = DistalDH([L1, 0, 0, theta2]);
    T23 = DistalDH([L2, 0, 0, theta3]);
    T34 = DistalDH([0, pi/2, 0, theta4]);
    T45 = DistalDH([0, 0, L3, theta5]);

    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;

    % each joint coord
    joint_positions = [
        0, 0, 0;                  % Base
        T01(1:3, 4)';             % Joint 1
        T02(1:3, 4)';             % Joint 2
        T03(1:3, 4)';             % Joint 3
        T04(1:3, 4)';             % Joint 4
        T05(1:3, 4)'              % End-effector
    ];
end

