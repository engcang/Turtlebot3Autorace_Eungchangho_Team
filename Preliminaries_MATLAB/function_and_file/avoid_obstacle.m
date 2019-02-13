function avoid_obstacle(desired_x, desired_y)
global robot lds velmsg odom

vfh = robotics.VectorFieldHistogram;
vfh.UseLidarScan=true;
vfh.DistanceLimits = [0.12 0.5];
vfh.RobotRadius = 0.105;
vfh.MinTurningRadius = 0.1;
vfh.SafetyDistance = 0.08;
vfh.HistogramThresholds = [2 5.5];


odomdata = receive(odom,3);
ax=odomdata.Pose.Pose.Position.X;
ay=odomdata.Pose.Pose.Position.Y;
%  targetDir = atan2(desired_y-ay, desired_x-ax); 
targetDir=0;
% while 1
while ax <= desired_x-0.05 && ay <= desired_y-0.05
    disp([ax ay]);
laserScan = receive(lds);
ranges = double(laserScan.Ranges);
angles = double(laserScan.readScanAngles);
scan = lidarScan(ranges, angles);
steerDir = vfh(scan,targetDir);

if ~isnan(steerDir)
		v = 0.05;
		w = exampleHelperComputeAngularVelocity(steerDir,1);
else 
		v = 0.0;
		w = 0.7;
end
velmsg.Linear.X = v;
velmsg.Angular.Z = w;
send(robot,velmsg);
odomdata = receive(odom,3);
ax=odomdata.Pose.Pose.Position.X;
ay=odomdata.Pose.Pose.Position.Y;
% targetDir = atan2(desired_y-ay, desired_x-ax);
disp([v w]);
if w==0
    break;
end
end
end