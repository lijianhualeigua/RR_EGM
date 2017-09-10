xbox = RobotRaconteur.Connect('rr+tcp://localhost:5437/?nodename=input_devices.xbox_controller&service=xbox_controller');
robot = RobotRaconteur.Connect('rr+tcp://localhost:62354/?service=EGM');
d = 0.005;
for i =1:100000000
    tic; 
    s=robot.joint_angle_setpoint;
    xBoxInput = xbox.controller_input;
    plus = xBoxInput.back_button;
    minus = xBoxInput.start_button;
    joint_1_inc = d*(double(xBoxInput.A));
    joint_2_inc = d*(double(xBoxInput.B));
    joint_3_inc = d*(double(xBoxInput.X));
    joint_4_inc = d*(double(xBoxInput.Y));
    joint_5_inc = d*(double(xBoxInput.left_button));
    joint_6_inc = d*(double(xBoxInput.right_button));
    if plus
        robot.joint_angle_setpoint=double([s(1)+joint_1_inc s(2)+joint_2_inc s(3)+joint_3_inc s(4)+joint_4_inc s(5)+joint_5_inc s(6)+joint_6_inc]');
    elseif minus
        robot.joint_angle_setpoint=double([s(1)-joint_1_inc s(2)-joint_2_inc s(3)-joint_3_inc s(4)-joint_4_inc s(5)-joint_5_inc s(6)-joint_6_inc]');
    else
    end
    t(i) =toc;
end