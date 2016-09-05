%% clear
clc
clear all
close all

tictoc=[];

%% Settings
% Optical Flow
opticalFlow = vision.OpticalFlow('Method','Horn-Schunck','ReferenceFrameDelay', 1); % ToDo:Explore the function posiblities
opticalFlow.OutputValue = 'Horizontal and vertical components in complex form';     % Sets a complex output for ploting
% Ploting
CircleInserter = vision.ShapeInserter('Shape','Circles','BorderColor','Custom', 'CustomBorderColor',uint8([255 0 0]));
BoxInserter = vision.ShapeInserter('Shape','Rectangles','BorderColor','Custom', 'CustomBorderColor',uint8([255 0 0]));
% Player
videoBoxes = vision.VideoPlayer('Name','Motion Vector','Position',[95 0 410 300]);
videoCircles = vision.VideoPlayer('Name','Motion Vector','Position',[95 350 410 300]);
videoBbox = vision.VideoPlayer('Name','Bbox');
videoMagnitud = vision.VideoPlayer('Name','Magnitud','Position',[515 383 410 300]);
videoOPThreshold = vision.VideoPlayer('Name','OPThreshold','Position',[515 0 410 300]);


%% SELECT VIDEO
VIDEO='Videos/VIDEO0159.mp4';
videoReader = vision.VideoFileReader(VIDEO ,'ImageColorSpace','RGB','VideoOutputDataType','uint8');  % ToDo:'VideoOutputDataType'???
[g,mask,rs,thr,AreaOF,radius,distCrossing]=videoloader(VIDEO);

% Number of frames
videoFrames = VideoReader(VIDEO);    % The VideoFileReader funtion isn't able to get the number of frames
frameNum=videoFrames.NumberOfFrames;


%% Position and Direction
Position = zeros(15,2*frameNum);      %Initialize the Position matrix
Direction = zeros(15,2*frameNum);     %Initialize the Direction matrix
theta = 0 : 0.05 : 2*pi;              %Angle Pixels Circle


%% LOOP
%First frame
frame0 = step(videoReader);         % Load a frame
frame0 = imresize(frame0,rs);       % Resize frame
frameSize = size(frame0);

%Video loop
k=2;
while ~isDone(videoReader)
    tic
    % SET FRAME
    frameRGB = step(videoReader);          % Load a frame
    frameRGB = imresize(frameRGB,rs);      % Resize frame
    
    frame = rgb2gray(frameRGB);            %Frame to gray
    frame = mat2gray(frame);

    [Circles,img]=FindCircles_noOF(frame,radius);
    TotalCircles=[Circles];

    tictoc=[tictoc; toc];
    k=1+k;
end
mean(tictoc)




%% RELEASE VIDEOS
% release(videoBoxes);
% release(videoCircles);
% release(videoMagnitud);
% release(videoOPThreshold);
% release(videoBbox);

release(videoReader);