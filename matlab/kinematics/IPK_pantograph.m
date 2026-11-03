function theta = IPK_pantograph(q, robot)

    x = q(1);
    y = q(2);

    l1 = robot.l1;
    l2 = robot.l2;
    l3 = robot.l3;
    l4 = robot.l4;
    d  = robot.d;

    A1 = [-d/2, 0];
    A2 = [ d/2, 0];
    P  = [x, y];

    % Left chain: A1-B1-P
    v1 = P - A1;
    r1 = norm(v1);

    if r1 > (l1 + l3) || r1 < abs(l1 - l3)
        error('Point unreachable by left chain');
    end

    alpha1 = atan2(v1(2), v1(1));

    c1 = (l1^2 + r1^2 - l3^2) / (2*l1*r1);
    c1 = max(min(c1, 1), -1);

    beta1 = acos(c1);

    theta1 = alpha1 + beta1;

    % Right chain: A2-B2-P
    v2 = P - A2;
    r2 = norm(v2);

    if r2 > (l2 + l4) || r2 < abs(l2 - l4)
        error('Point unreachable by right chain');
    end

    alpha2 = atan2(v2(2), v2(1));

    c2 = (l2^2 + r2^2 - l4^2) / (2*l2*r2);
    c2 = max(min(c2, 1), -1);

    beta2 = acos(c2);

    theta2 = alpha2 - beta2;

    theta = [theta1; theta2];

end