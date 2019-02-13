function [ binary_image ] = traffic(img,g)
    binary_image = zeros(480,640);
    for row=170:240
        for col=300:465             
            if 21<= img(row,col,1) && img(row,col,1) <= 47 && 40<= img(row,col,2) && img(row,col,2) <= 85 && 24<= img(row,col,3) && img(row,col,3) <= 58 && 12<= g(row,col) && g(row,col) <=50
               binary_image(row,col)=255;           
            end    
        end
    end
end