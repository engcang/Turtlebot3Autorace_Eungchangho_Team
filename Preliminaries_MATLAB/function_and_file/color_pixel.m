function [pixel] = color_pixel(image)
    [R, C] = size(image);
    pixel = 0;
    for row=1:R
        for col=1:C   
            value = image(row,col);
            if value ~= 0                    
                pixel = pixel + 1;
            end
        end
    end

end

