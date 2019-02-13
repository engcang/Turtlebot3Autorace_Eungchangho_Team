function parking()
global sub_img robot lds odom tbot velmsg never_parking
while never_parking == 0
    disp('Parking');
    resetOdometry(tbot);
    pause(1);
    odomdata = receive(odom);
    pose = odomdata.Pose.Pose;

% ���� ����� �԰ݿ� ���� �ٲ� ��
    first_check = 0.49;
    parking_check = 0.27; % ���Ÿ�
    second_check = 0.98; %
    never_parking = 0;
    %% ó�� ���� ���� ������ �� ���� ����
    velmsg.Linear.X = 0.10;
    send(robot,velmsg);
     
      while  pose.Position.Y < first_check
          velmsg.Linear.X = 0.10;
          send(robot,velmsg);
          odomdata = receive(odom,3);
          pose = odomdata.Pose.Pose;
          img_tmp = receive(sub_img);
          img_tmp.Format = 'bgr8;';
          img_raspi = readImage(img_tmp);
          [r, g, b, y]=real_rgby(img_raspi);
          keeping(img_raspi,y);
      end
      reset = pose.Position.X; % �������������� 0���� ���ֱ� ����
      stop();
%% ó�� �������� ���� �� ���� or ����ħ
      park_check=5*ones(360,1);
      lds_temp = receive(lds);
for i = 240:300
    if lds_temp.Ranges(i)~=0
    park_check(i)=lds_temp.Ranges(i);
    end
end
[min_dist,min_indx]=min(park_check);
if min_dist < 0.35 && 240<=min_indx && min_indx<=300
disp('No-parking-pot');
    velmsg.Linear.X = 0.05;
    send(robot,velmsg);
else % ���� �� ��Ȳ
      rotate(-90);
      
     velmsg.Linear.X = 0.05;
     send(robot,velmsg);
      while pose.Position.X-reset  < parking_check % ǥ���� ���� �� �ڽ��� Y���� reset_Y ���� ���� // pose.Position.Y - reset_Y�� 0 �� ������ ������������
               odomdata = receive(odom,3);
                pose = odomdata.Pose.Pose;
      end
      stop();
      pause(2);
     velmsg.Linear.X = -0.05;
     send(robot,velmsg);
      while pose.Position.X-reset > -0.02    % ǥ���� ���� �� �ڽ��� Y���� reset_Y ���� ���� // pose.Position.Y - reset_Y�� 0 �� ������ ������������
     odomdata = receive(odom,3);
     pose = odomdata.Pose.Pose;
      end   
      stop();
      rotate(90)
      never_parking = 1;
      break;
end
    %% 2��° ������������ ����
     velmsg.Linear.X = 0.05;
     send(robot,velmsg);
    %% 2��° �������� ���� �� ���� or ����ħ   
 while  pose.Position.Y < second_check
    odomdata = receive(odom,3);
    pose = odomdata.Pose.Pose;
    img_tmp = receive(sub_img);
    img_tmp.Format = 'bgr8;';
    img_raspi = readImage(img_tmp);
    [r, g, b, y]=real_rgby(img_raspi);
    figure(1),imshow(y);
    keeping(img_raspi,y);
 end
stop();

 park_check=5*ones(360,1);
 lds_temp = receive(lds);
for i = 240:300
    if lds_temp.Ranges(i)~=0
    park_check(i)=lds_temp.Ranges(i);
    end
end
[min_dist,min_indx]=min(park_check);
if min_dist < 0.35 && 240<=min_indx && min_indx<=300
disp('None');
     velmsg.Linear.X = 0.05;
     send(robot,velmsg);
else % ���� �� ��Ȳ
    if never_parking == 0
     rotate(-90);
     
     velmsg.Linear.X = 0.05;
     send(robot,velmsg);
      while pose.Position.X-reset  < parking_check
               odomdata = receive(odom,3);
                pose = odomdata.Pose.Pose;
      end
      stop();
      pause(2);
     velmsg.Linear.X = -0.05;
     send(robot,velmsg);
      while pose.Position.X-reset > - 0.02
     odomdata = receive(odom,3);
     pose = odomdata.Pose.Pose;
      end   
      stop();
      rotate(90);
       never_parking = 1;
       break;
    end
end
end
end