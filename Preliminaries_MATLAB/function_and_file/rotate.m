function rotate(int)
global velmsg robot
degree=int/180*pi;

if int < 0
velmsg.Angular.Z=-1;
velmsg.Linear.X = 0;
elseif int >0
velmsg.Angular.Z=1;
velmsg.Linear.X = 0;
end
send(robot,velmsg);
pause(abs(degree));
stop();
end