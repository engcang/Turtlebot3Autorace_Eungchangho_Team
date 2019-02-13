function [ binary_image ] = tunnel_y(real_y)
    [R, C] = size(real_y);  
    binary_image = zeros(R,C);

    for row=100:240 
        for col=100:300 
            if real_y(row,col) >= 70 
               binary_image(row,col)=255;           
            end    
        end
    end
end