function result = NotOverlap(x,y,width,height,rect)
result = 1;
% initial 
if isempty(rect)
    result = 1;
% detect whether (x,y,width,height) is overlapped with other rectangle     
else
    rect_size = size(rect,1);
    for i = 1:rect_size
        tmp = abs(x + width/2 - rect(i,1) - rect(i,3)/2 ) < abs(width + rect(i,3))/2 && ...
              abs(y + height/2 - rect(i,2) - rect(i,4)/2) < abs(height + rect(i,4))/2;
        result = result && ~tmp;
        if result == 0
            break;
        end
    end
end   
end