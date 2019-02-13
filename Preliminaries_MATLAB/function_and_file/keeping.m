function keeping(img_raspi,y)
global img robot velmsg never_parking
%% variables set
    check_line_L = zeros(480,640);
    check_line_R = zeros(480,640);
%% line check and binary image
      for i=350:410
        for j=40:280
            if img_raspi(i,j,1) > 120 && img_raspi(i,j,2) > 120 && img_raspi(i,j,3) > 120
                check_line_L(i,j) = 255;
            end
            if y(i,j) > 40
                check_line_L(i,j) = 255;
            end
        end
        for j=390:580
            if img_raspi(i,j,1) > 120 && img_raspi(i,j,2) > 120 && img_raspi(i,j,3) > 120
                check_line_R(i,j) = 255;
            end
            if y(i,j) > 40
                check_line_R(i,j) = 255;
            end
        end
      end
      
      figure(1),imshow(check_line_R);
      figure(2),imshow(check_line_L);

%% yello line and white line detect
     check_line_R = imresize(check_line_R, 0.2);
     check_line_L = imresize(check_line_L, 0.2);
    left_line = img.detect(check_line_L);
    right_line = img.detect(check_line_R);
%% moving to keep line
velmsg.Linear.X=0.1;
send(robot,velmsg);
if never_parking == 0
%     if isempty(right_line)
%         rotate(-15);
%     end
%     if isempty(left_line)
%         rotate(15);
%     end
    if isempty(left_line) % new method
        rotate(15);
    elseif isempty(right_line)
        rotate(-15);
    end
else
    if isempty(right_line)
        rotate(-15);
    elseif isempty(left_line)
        rotate(15);
    end
end
end