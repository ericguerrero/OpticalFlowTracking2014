function [Circles,Ibox]=FindCircles(Bbox,frame, radii)
    x=round(Bbox(2)+1);x2=round(Bbox(2)+Bbox(4)-2);% Coordenates of the moving area
    y=round(Bbox(1)+1);y2=round(Bbox(1)+Bbox(3)-2);
    Circles=[];
    Ibox = frame(x:x2,y:y2);% Pieze of the original image
    Ibox = edge(Ibox,'sobel');% Edge detector to get more accuracy in the following function
    
    [centers radii] = imfindcircles(Ibox,radii,'Sensitivity',0.9); % Find circles function
    if ~isempty(centers)
        Circles=[centers(:,1)+y,centers(:,2)+x,radii];
    end
end