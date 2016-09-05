%% Design and implementation of a technique to track moving objects based on optical flow

%% clear
clc
clear all
close all

tictoc=[];

%% Settings
% Optical Flow
opticalFlow = vision.OpticalFlow('Method','Horn-Schunck','ReferenceFrameDelay', 1); % Sets the Horn-Schunck optical flow between two consecutive frames
opticalFlow.OutputValue = 'Horizontal and vertical components in complex form';     % Sets a complex output for ploting
% Ploting
LineInserter = vision.ShapeInserter('Shape','Lines','BorderColor','Custom', 'CustomBorderColor',uint8([0 0 255]));  % Type of shapes to insert for visualization
CircleInserter = vision.ShapeInserter('Shape','Circles','BorderColor','Custom', 'CustomBorderColor',uint8([255 0 0]));
BoxInserter = vision.ShapeInserter('Shape','Rectangles','BorderColor','Custom', 'CustomBorderColor',uint8([0 255 0]));
% Player
videoShapes = vision.VideoPlayer('Name','Shapes','Position',[95 0 410 300]);    % To show the shapes
videoBbox = vision.VideoPlayer('Name','Bbox');                                  % To show the inside of a bounding box after sobel is applied
videoMagnitud = vision.VideoPlayer('Name','Magnitud','Position',[95 383 410 300]);% Magnitude of the complex optical flow image
videoOPThreshold = vision.VideoPlayer('Name','OPThreshold','Position',[515 0 410 300]);% Movement Threshold
% Paths
addpath(genpath('Videos/'))
addpath(genpath('Functions/'))
addpath(genpath('Comparisons/'))

%% SELECT VIDEO
VIDEO='Videos/VIDEO0159.mp4';   % Video path  Uncoment if you are not using the interface
videoReader = vision.VideoFileReader(VIDEO ,'ImageColorSpace','RGB','VideoOutputDataType','uint8'); % Sets the video to read
[g,mask,rs,thr,AreaOF,radius,distCrossing,pixelsize]=videoloader(VIDEO);    %Get some data for the video

% Number of frames
videoFrames = VideoReader(VIDEO);    % The VideoFileReader funtion isn't able to get the number of frames, so we use this one
frameNum=videoFrames.NumberOfFrames;


%% Position and Direction
Position = zeros(15,2*frameNum);      %Initialize the Position matrix
Direction = zeros(15,2*frameNum);     %Initialize the Direction matrix
theta = 0 : 0.05 : 2*pi;              %Angle Pixels Circle


%% LOOP
%First frame
frame0 = step(videoReader);         % Load a frame
frame0 = imresize(frame0,rs);       % Resize frame

%Video loop
k=2;
while ~isDone(videoReader)
    tic
    % SET FRAME
    frameRGB = step(videoReader);          % Load a frame
    frameRGB = imresize(frameRGB,rs);      % Resize frame
    
    frame = rgb2gray(frameRGB);            %Frame to gray
    frame = mat2gray(frame);
    
    % OPTICAL FLOW
    frameOF = step(opticalFlow, frame);      % Optical flow
    frameOF = imfilter(frameOF,g);           % Gaussian Filtering


    % MORPHOLOGICAL
    frameMag = abs(frameOF);                        % Magnitude
    frameOFThr = im2bw(frameMag, thr);              % Threshold
    frameOFThr = imclose(frameOFThr,mask);            % Closing
    frameOFThr = imfill(frameOFThr,'holes');          % Fill
    frameOFThr = imopen(frameOFThr,mask);             % Opening
    
    % LABELING (moving areas)
    [Labels, NUM] = bwlabel(frameOFThr); % Labeling of the moving areas
    Features = regionprops(Labels,'Area', 'BoundingBox');   % Region Features
    
    % Bounding box
    Boxes=zeros(NUM,4);
    TotalCircles=[];
    for i=1:NUM
        Area=[Features(i,:).Area];              % Get the area
        if Area>=AreaOF   % If the area is less don't search for de boxes and cercles.
            Bbox=[Features(i,:).BoundingBox];   % Get the bounding box
            Boxes=[Boxes; Bbox];                %Boxes per frame
            % CIRCLES
            [Circles,img]=FindCircles(Bbox,frame,radius);%Find the objects in each bounding box
            TotalCircles=[TotalCircles; Circles];% Build a vector objects
        end    
    end
    NumCir=size(TotalCircles,1);% Objects per frame
    
    % POSITION AND DIRECTION
    if ~isempty(TotalCircles)
        Position(1:NumCir,2*k-1:2*k) = TotalCircles(:,1:2);%Build the positions vector
        for i=1:NumCir %For all the objects, get the direction as the average of the pixels directions
            pixelsX = ceil((3*TotalCircles(i,3)/4:TotalCircles(i,3))' * cos(theta) + Position(i,2*k-1));
                pixelsX = reshape(pixelsX,1,[]);
            pixelsY = ceil((3*TotalCircles(i,3)/4:TotalCircles(i,3))' *sin(theta)+Position(i,2*k));
                pixelsY = reshape(pixelsY,1,[]);
            sof=size(frameOF);
            for j=1:length(pixelsY)
                if pixelsY(j)>=sof(1)
                    pixelsY(j)=sof(1);
                end
                if pixelsX(j)>=sof(2)
                    pixelsX(j)=sof(2);
                end
                if pixelsY(j)<=1
                    pixelsY(j)=1;
                end
                if pixelsX(j)<=1
                    pixelsX(j)=1;          
                end
            end
            dir=frameOF(sub2ind(size(frameOF),pixelsY,pixelsX));
            Direction(i,2*k-1) = mean(dir);
        end
        %Online representation of position and direction in the interface
        
        quiver(TotalCircles(:,1),-TotalCircles(:,2),real(Direction(1:NumCir,2*k-1))...
            ,-imag(Direction(1:NumCir,2*k-1)),0.05,'color',[0 0 1]);hold on;
        xlim([0 384]);ylim([-216 0 ]);title('Position and Velocity'); 
    end  
    

     %% VIDEO
    frameshapes = repmat(frame,[1 1 3]); % Frame to RGB for plot with colors
    %Lines
    lines = videooptflowlines(frameOF, 100); 
    if ~isempty(lines)
        out = step(LineInserter, frameshapes, lines); 
    end
    %Boxes
    if ~isempty(Boxes)
        out = step(BoxInserter, out, Boxes);
    end
    %Circles
    if ~isempty(TotalCircles)
        out = step(CircleInserter, out, TotalCircles);
    end
    step(videoShapes, out); %Reproduce the frame in the player
    
%     figure(2);
%     imagesc(frameMag)
%     colormap(jet)
%     colorbar;
    step(videoMagnitud, frameMag);
    step(videoOPThreshold, frameOFThr);
% 
%     release(videoBbox);
%     step(videoBbox, img)  
    tictoc=[tictoc; toc];
    k=1+k;
end
mean(tictoc)



%% Trajectory Generation
TrajThr=frameNum/10;
[Traj, Orien, NumTraj]=MTG(Position,Direction,distCrossing,TrajThr);% MultiTrajectory Generation

%% Trajectory Ploting
MTP(Traj, Orien, NumTraj,pixelsize,frame)% MultiTrajectory Ploting

%% RELEASE VIDEOS
release(videoShapes);
release(videoMagnitud);
release(videoOPThreshold);
% release(videoBbox);

release(videoReader);

