function [real_r,real_g,real_b,real_y]=real_rgby(rgb_img)
    r = rgb_img(:,:,1);
    g = rgb_img(:,:,2);
    b = rgb_img(:,:,3);
    r = double(r);
    g = double(g);
    b = double(b);    
    real_r = r-(g+b)/2;
    real_g = g-(r+b)/2;
    real_b = b-(r+g)/2;
    real_y = zeros(480,640);
    for i=1:480
        for j=1:640
            if (r(i,j)-g(i,j))>0
                real_y(i,j) = (r(i,j)+g(i,j))/2 - (r(i,j)-g(i,j))/2 - b(i, j); 
            else
                real_y(i,j) = (r(i,j)+g(i,j))/2 - (g(i,j)-r(i,j))/2 - b(i, j);
            end
        end
    end
    real_r = uint8(real_r);
    real_g = uint8(real_g);
    real_b = uint8(real_b);
    real_y = uint8(real_y);
end