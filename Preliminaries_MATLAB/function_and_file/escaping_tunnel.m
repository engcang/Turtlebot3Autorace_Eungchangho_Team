global robot odom tbot
stop();
resetOdometry(tbot);
pause(2.5); % time for completing the reset of turtlebot's odometry

info_a = receive(odom);
info_b = info_a.Pose.Pose;
robot_x = info_b.Position.X;
robot_y = info_b.Position.Y;

temp_x = robot_x + 0.75; % entering distance after recognizing the sign board
desired_x1 = temp_x + 1.1; % total x length of tunnel + entering distance after recognizing the sign board
desired_y1 = robot_y + 0.1; % total y length of tunnel 
desired_x2 = temp_x + 0.8;
desired_y2 = robot_y + 1.3;
desired_x3=temp_x+1.35;
desired_y3=robot_y+1.35;
reach_simple(temp_x,0); % without avoiding obstacles
reach(desired_x1,desired_y1)
reach(desired_x2, desired_y2); % passing tunnel with avoiding obstacles
reach_simple(desired_x3,desired_y3); %lotto
angle90; % rotating tutlebot for the exit

velmsg.Linear.X = 0.1;
velmsg.Angular.Z = 0;
send(robot,velmsg); 