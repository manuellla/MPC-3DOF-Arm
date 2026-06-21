function dxdt = arm_state(x, u)
    persistent robot
    if isempty(robot)
        robot = evalin('base','robot');
    end

    x = x(:);
    u = u(:);

    q  = x(1:3)';
    qd = x(4:6)';

    qdd = robot.accel(q, qd, u');

    dxdt = [qd(:); qdd(:)];   % 3x1 + 3x1 vertcat = 6x1, robusto a orientacao de qd/qdd
end