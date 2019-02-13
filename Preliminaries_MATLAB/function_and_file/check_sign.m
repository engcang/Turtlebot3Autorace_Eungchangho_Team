function [f]=check_sign(img_raspi,r,g,b,y)

detector_stop = vision.CascadeObjectDetector('stop2.xml');
detector_parking = vision.CascadeObjectDetector('parking.xml');
detector_tunnel = vision.CascadeObjectDetector('tunnel1.xml');

bbox_stop = step(detector_stop,img_raspi);
bbox_parking = step(detector_parking,img_raspi);
bbox_tunnel = step(detector_tunnel,img_raspi);
f = 5;

if bbox_stop ~= 0
    if  color_pixel_box(stop_r(r),bbox_stop) >= 900
        f = 2;
    end
elseif bbox_parking ~= 0
    if color_pixel_box(parking_b(b),bbox_parking) >= 950
        f = 1;
    end
elseif bbox_tunnel ~= 0
    if color_pixel_box(tunnel_y(y),bbox_tunnel) >= 150
        f = 4;
    end
else
     if 62 >= color_pixel(traffic(img_raspi,g)) && color_pixel(traffic(img_raspi,g)) >=35
         f = 3;
     end
end