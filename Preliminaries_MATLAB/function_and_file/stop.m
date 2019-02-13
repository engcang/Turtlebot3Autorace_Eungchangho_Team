function stop()
global robot velmsg
velmsg.Linear.X=0;
velmsg.Angular.Z=0;
send(robot,velmsg);
end