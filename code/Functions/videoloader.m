%%Video loader%%

function [g,mask,rs,thr,AreaOF,radius,distCrossing,pixelsize]=videoloader(Name)

if strcmp(Name,'Videos/VIDEO0156.mp4')||strcmp(Name,'Videos/VIDEO0158.mp4');
    typeofvideo=1;
elseif strcmp(Name,'Videos/VIDEO0159.mp4');
    typeofvideo=2;
elseif strcmp(Name,'Videos/VIDEO0170.mp4')||strcmp(Name,'Videos/VIDEO0168.mp4')|| ...
       strcmp(Name,'Videos/VIDEO0166.mp4');
    typeofvideo=3;
elseif strcmp(Name,'Videos/VIDEO0172D.mp4')|| ...
       strcmp(Name,'Videos/VIDEO0184.mp4')||strcmp(Name,'Videos/VIDEO0185.mp4');
    typeofvideo=4;
end

g = fspecial('gaussian',15,0.8);% Gaussian Mask (Original frame and OpticalFlow frame (of)) 31
mask = strel('disk',15);           % Closing Opening Mask 21
rs = 0.2;                       % Frame resize
AreaOF = 4000;
radius = [25 40];
distCrossing = 35;


switch typeofvideo
%% VIDEO 27/11    
    case 1
distCrossing = 30;
thr = 0.03;

%% VIDEO 28/11
    case 2
thr = 0.02;

%% VIDEO 01/12
    case 3
thr = 0.015;

%% VIDEO 03/12
    case 4
rs = 1;                       % Frame resize
thr = 0.015;
radius = [25 55];

end
pixelsize = 222*rs/65;             % pixels/mm Diam of the balls reference of 222px,65mm
end
