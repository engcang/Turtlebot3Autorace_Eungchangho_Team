function [ binary_image ] = traffic_g(real_g)
    [R, C] = size(real_g);  
    binary_image = zeros(R,C);
    for row=1:R
        for col=C/2:C
            if real_g(row,col) >= 30
               binary_image(row,col)=255;           
            end    
        end
    end
end