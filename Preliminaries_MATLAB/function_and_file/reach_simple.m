function reach_simple(x,y)
global robot velmsg
while 1
[v, w] = destination(x,y);
if v == 0 && w == 0
    break;
end
disp("in");
velmsg.Linear.X = v;
velmsg.Angular.Z = w;
send(robot,velmsg); 
end