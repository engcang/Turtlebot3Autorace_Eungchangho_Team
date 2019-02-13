function [v, w] = destination(x,y)
global odom
info_a = receive(odom);
info_b = info_a.Pose.Pose;
robot_x = info_b.Position.X;
robot_y = info_b.Position.Y;
robot_ox = info_b.Orientation.X;
robot_oy = info_b.Orientation.Y;
robot_oz = info_b.Orientation.Z;
robot_ow = info_b.Orientation.W;

temp = [robot_ow robot_ox robot_oy robot_oz];
theta = quat2eul(temp);
theta = theta(1);
theta = theta * 180 / pi;

difference_x = x - robot_x;
difference_y = y - robot_y;

alpha = atan2(difference_y, difference_x);
alpha = alpha * 180 / pi;

difference_angle = alpha - theta;
if difference_angle < - 180
    difference_angle = difference_angle + 360;
elseif difference_angle > 180
    difference_angle = difference_angle - 360;
end  

difference_distance = sqrt(difference_x*difference_x + difference_y*difference_y);

disp([robot_x robot_y]);
if difference_distance < 0.07
    v = 0;
    w = 0;
else
    if difference_angle > 15 || difference_angle < -15
        if difference_angle > 0 
            v = 0;
            w = 0.8;   %경기장값
        else
            v = 0;
            w = -0.8;  %경기장값
        end
    else
        difference_angle = difference_angle * pi / 180;
        v = 0.1;
        w = 3 * difference_angle;  %경기장값
        if w >= 0.8
            w = 0.8;
        elseif w<= -0.8
            w = -0.8;
        end
    end
end           