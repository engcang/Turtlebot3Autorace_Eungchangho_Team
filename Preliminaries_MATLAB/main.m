%% main code
% rosshutdown
% clear all
% clc

addpath('./function_and_file');

% rosinit('192.168.0.20');

global robot velmsg sub_img img odom tbot lds never_parking never_traffic stop_pass
never_parking = 0;
never_traffic = 0;
stop_pass = 0;
sub_img = rossubscriber('/raspicam_node/image/compressed');
robot = rospublisher('/cmd_vel');
    velmsg = rosmessage(robot);
lds = rossubscriber('/scan');
odom = rossubscriber('/odom');
%     tbot = turtlebot;
%     tbot.OdometryReset.TopicName = '/reset';
    resetOdometry(tbot);
pause(2);
img = cv.LineSegmentDetector();

while 1
        img_tmp = receive(sub_img);
        img_tmp.Format = 'bgr8;';
        img_raspi = readImage(img_tmp);
   [r, g, b, y]=real_rgby(img_raspi); 
   [f]=check_sign(img_raspi,r,g,b,y); 
   if stop_pass == 0
    keeping(img_raspi,y); 
   end
disp(f);
   switch f
       case 1   % parking sign
           if never_parking == 0
           parking; %(라이다값)
           end
       case 2   % stop sign
           bar_action; %(하기 전후 lane keeping)
       case 3   % traffic signal 
           if never_traffic ==0
           traffic_action;  %(초록색 값 확인해보기)
           end
       case 4   % Tunnel
           escaping_tunnel; %(경기장값해보기)
       case 5   % nothing, lane keeping
           velmsg.Linear.X = 0.08;
           velmsg.Angular.Z= 0;
           send(robot,velmsg);
   end
   
end