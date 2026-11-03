function q = FPK_pantograph(theta, robot)

    theta1 = theta(1);
    theta2 = theta(2);

    l1 = robot.l1;
    l2 = robot.l2;
    l3 = robot.l3;
    l4 = robot.l4;
    d  = robot.d;

    % First moving joints
    xB1 = -d/2 + l1*cos(theta1);
    yB1 =        l1*sin(theta1);

    xB2 =  d/2 + l2*cos(theta2);
    yB2 =        l2*sin(theta2);

    % Distance between B1 and B2
    D = sqrt((xB2 - xB1)^2 + (yB2 - yB1)^2);

    % Check if circles intersect
    if D > l3 + l4 || D < abs(l3 - l4)
        error('No valid FPK solution');
    end

    % Circle intersection
    a = (l3^2 - l4^2 + D^2)/(2*D);
    h = sqrt(l3^2 - a^2);

    xH = xB1 + a*(xB2 - xB1)/D;
    yH = yB1 + a*(yB2 - yB1)/D;

    % Solution 1
    xP1 = xH - h*(yB2 - yB1)/D;
    yP1 = yH + h*(xB2 - xB1)/D;

    % Solution 2
    xP2 = xH + h*(yB2 - yB1)/D;
    yP2 = yH - h*(xB2 - xB1)/D;

    % Output BOTH solutions
    q = [xP1  xP2;
         yP1  yP2];

end