global robot velmsg
% lds = rossubscriber('/scan');
info = rossubscriber('/odom');

while 1
info_a = receive(info);
info_b = info_a.Pose.Pose;
robot_ox = info_b.Orientation.X;
robot_oy = info_b.Orientation.Y;
robot_oz = info_b.Orientation.Z;
robot_ow = info_b.Orientation.W;

temp = [robot_ow robot_ox robot_oy robot_oz];
theta = quat2eul(temp);

theta = theta(1);
theta = theta * 180 / pi;

theta = theta + 180;

if theta > 271
    w = -1;
elseif theta <269.5
    w = 1;
else
    velmsg.Angular.Z = 0;
    send(robot,velmsg); 
    break;
end

disp(theta);
disp(w);

velmsg.Linear.X = 0;
velmsg.Angular.Z = w;
send(robot,velmsg); 
end
% end
