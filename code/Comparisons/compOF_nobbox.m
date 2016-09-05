%% clear
clc
clear all
close all
tictoc=[];

%% Settings
% Optical Flow
opticalFlow = vision.OpticalFlow('Method','Horn-Schunck','ReferenceFrameDelay', 1); % ToDo:Explore the function posiblities
opticalFlow.OutputValue = 'Horizontal and vertical components in complex form';     % Sets a complex output for ploting
% Complex Ploting
CircleInserter = vision.ShapeInserter('Shape','Circles','BorderColor','Custom', 'CustomBorderColor',uint8([255 0 0]));
BoxInserter = vision.ShapeInserter('Shape','Rectangles','BorderColor','Custom', 'CustomBorderColor',uint8([255 0 0]));
% Player
videoVector1 = vision.VideoPlayer('Name','Motion Vector','Position',[95 0 410 300]);
videoVector2 = vision.VideoPlayer('Name','Motion Vector','Position',[95 350 410 300]);

videoBbox = vision.VideoPlayer('Name','Bbox','Position',[95 383 410 300]);
videoMagnitud = vision.VideoPlayer('Name','Magnitud','Position',[515 383 410 300]);
videoOPThreshold = vision.VideoPlayer('Name','OPThreshold');
videoFrame = vision.VideoPlayer('Name','Frame');

%% SELECT VIDEO
VIDEO='Videos/VIDEO0159.mp4';
videoReader = vision.VideoFileReader(VIDEO ,'ImageColorSpace','RGB','VideoOutputDataType','uint8');  % ToDo:'VideoOutputDataType'???

[g,mask,rs,thr,AreaOF,radius]=videoloader(VIDEO);
%% Position, Velocity and Acceleration
Position = zeros(6,800);      %Initialize the Position
Direction = zeros(6,800);     %Initialize the Direction
theta = 0 : 0.05 : 2*pi;        %Angle Pixels Circle
linespec = {'b', 'r', 'g'};
legendInfo = {'a'};

%% LOOP
%First frame
frame0 = step(videoReader);         % Load a frame
frame0 = imresize(frame0,rs);       % Resize frame
S=size(frame0);


nc=[];
k=2;
while ~isDone(videoReader)
    tic
    % Set frame
    frameRGB = step(videoReader);           % Load a frame
    frameRGBr = imresize(frameRGB,rs);      % Resize frame
    
    frame = rgb2gray(frameRGBr);    %Frame to gray
    frame = mat2gray(frame);
    
    % Optical Flow
    of = step(opticalFlow, frame);      % Optical flow
    of = imfilter(of,g);                % Gaussian Filtering

    % Morphological
    magagnitude = abs(of);                  % Magnitude
    OFThr = im2bw(magagnitude, thr);        % Threshold
    OFThr = imclose(OFThr,mask);            % Closing
    OFThr = imfill(OFThr,'holes');          % Fill
    OFThr = imopen(OFThr,mask);             % Opening
    
    % Moving zones labeling
    

    Image=OFThr.*frame;

    Circles=[];
    Ibox = Image;% Pieze of the original image
    Ibox = edge(Ibox,'sobel');% Edge detector to get more accuracy in the following function
    
    [centers radii] = imfindcircles(Ibox,radius,'Sensitivity',0.9); % Find circles function
    if ~isempty(centers)
        TotalCircles=[centers(:,1),centers(:,2),radii];
    end

tictoc=[tictoc; toc];
end
mean(tictoc)


%% CLOSE VIDEO

% release(videoBbox);

release(videoReader);