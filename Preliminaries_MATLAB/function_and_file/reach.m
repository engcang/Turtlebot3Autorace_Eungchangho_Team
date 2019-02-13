function reach(desired_x, desired_y)
global robot velmsg
while 1
avoid_obstacle(desired_x, desired_y);
[v, w] = destination(desired_x, desired_y);
if v == 0 && w == 0
    break;
end
disp("in2");
velmsg.Linear.X = v;
velmsg.Angular.Z = w;
send(robot,velmsg); 
end