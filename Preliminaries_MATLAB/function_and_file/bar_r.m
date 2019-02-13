function [ binary_image ] = bar_r(real_r)
    binary_image = zeros(480,640);
    
    for row =100 : 240
        for col=50:550
            if real_r(row,col) >= 85
               binary_image(row,col)=255;           
            end    
        end
    end
end