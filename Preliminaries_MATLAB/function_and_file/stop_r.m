function [ binary_image ] = stop_r(real_r)
    [R, C] = size(real_r);  
    binary_image = zeros(R,C);
    for row=80:300
        for col=C/2:C
            if real_r(row,col) >= 70
               binary_image(row,col)=255;           
            end    
        end
    end
end