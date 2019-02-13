global sub_img robot velmsg stop_pass

while 1
img_tmp = receive(sub_img);
img_tmp.Format = 'bgr8;';
img_raspi = readImage(img_tmp);
[r, g, b, y]=real_rgby(img_raspi);

    if  color_pixel(bar_r(r)) >= 2200 % judging the state only using the number of bar's red color
        velmsg.Linear.X = 0;
        velmsg.Angular.Z= 0;
        send(robot,velmsg);
    else
        velmsg.Linear.X = 0.02;  % 무한정지?
        send(robot,velmsg);
        stop_pass = 1;
        break;
    end
end