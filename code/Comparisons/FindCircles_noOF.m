function [Circles,Ibox]=FindCircles(frame, radii)
    
    Circles=[];
    Ibox = edge(frame,'sobel');% Edge detector to get more accuracy in the following function
    [centers radii] = imfindcircles(Ibox,radii,'Sensitivity',0.9); % Find circles function
    if ~isempty(centers)
        Circles=[centers(:,1),centers(:,2),radii];
    end
end