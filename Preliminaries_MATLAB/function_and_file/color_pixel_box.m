function [pixel] = color_pixel_box(image,bbox)
    pixel = 0;
    if size(bbox,1)>1
        area = bbox(:,3).*bbox(:,4);
        [Area, I]= max(area);
    
        for row=bbox(I,2):bbox(I,2)+bbox(I,4)
            for col=bbox(I,1):bbox(I,1)+bbox(I,3)   
                value = image(row,col);
                if value ~= 0                    
                    pixel = pixel + 1;
                end
            end
        end
    else
        for row=bbox(1,2):bbox(1,2)+bbox(1,4)
            for col=bbox(1,1):bbox(1,1)+bbox(1,3)
                value = image(row,col);
                if value ~= 0                    
                    pixel = pixel + 1;
                end
            end
        end
    end
end

