function qdd = plant_dynamics(in)
    q  = in(1:3);
    qd = in(4:6);
    u  = in(7:9);

    persistent robot
    if isempty(robot)
        robot = evalin('base', 'robot');
    end
    qdd_row = robot.accel(q', qd', u');
    qdd = qdd_row';
end