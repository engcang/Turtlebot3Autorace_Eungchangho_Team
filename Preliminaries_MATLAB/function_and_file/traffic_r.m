function [ binary_image ] = traffic_r(real_r)
    [R, C] = size(real_r);  
    binary_image = zeros(R,C);
    for row=1:R
        for col=C/2:C
            if real_r(row,col) >= 80
               binary_image(row,col)=255;           
            end    
        end
    end
end