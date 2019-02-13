global robot velmsg never_traffic

velmsg.Linear.X=0.05;
velmsg.Angular.Z=0;
send(robot,velmsg);

pause(5);
stop();
pause(5.5);

never_traffic = 1;

velmsg.Linear.X=0.03;
velmsg.Angular.Z=0;
send(robot,velmsg);