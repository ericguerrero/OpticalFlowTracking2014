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
% Player
videoCircles = vision.VideoPlayer('Name','Motion Vector','Position',[95 350 410 300]);
%% SELECT VIDEO
VIDEO='Videos/VIDEO0159low.avi';
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
frameSize = size(frame0);

%Video loop
k=2;
while ~isDone(videoReader)
    tic
    % SET FRAME
    frameRGB = step(videoReader);          % Load a frame
   
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
    
    % LABELING (input:frameOFThr   output:TotalCircles
    [Labels, NUM] = bwlabel(frameOFThr); % Labeling of the moving areas
    Features = regionprops(Labels,'Area', 'BoundingBox');   % Region Features
    
    % BOUNDING BOX
    TotalCircles=[];
    for i=1:NUM
        Area=[Features(i,:).Area];
        if Area>=AreaOF   % If the area es less don't search for de boxes and cercles.
            Bbox=[Features(i,:).BoundingBox];
            % CIRCLES
            [Circles,img]=FindCircles(Bbox,frame,radius);
            TotalCircles=[TotalCircles; Circles];
        end    
    end
    NumCir=size(TotalCircles,1);
    % POSITION AND DIRECTION
    if ~isempty(TotalCircles)
        Position(1:NumCir,2*k-1:2*k) = TotalCircles(:,1:2);
        for i=1:NumCir
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
        %%COMENTANT RETALLAM 0.05 DE 0.028
        
        frameCircle = repmat(frame,[1 1 3]); % Frame to RGB for plot the red circles
        out2 = step(CircleInserter, frameCircle, TotalCircles);
        step(videoCircles, out2);
    end  
    
    tictoc=[tictoc; toc];
    k=1+k;
end
mean(tictoc)


%% Trajectory Generation
[Traj, Orien, NumTraj]=MTG(Position,Direction,distCrossing);% MultiTrajectory Generation

%% Trajectory Ploting
MTP(Traj, Orien, NumTraj)% MultiTrajectory Ploting

%% RELEASE VIDEOS
release(videoCircles);
release(videoReader);