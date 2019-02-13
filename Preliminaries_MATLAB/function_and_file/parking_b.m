function [ binary_image ] = parking_b(real_b)
    [R, C] = size(real_b);  
    binary_image = zeros(R,C);
    for row=1:R
        for col=C/2:C
            if real_b(row,col) >= 28
               binary_image(row,col)=255;
            end    
        end
    end
end