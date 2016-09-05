%% clear
clc
clear all
close all

tictoc=[];
%% Settings
% Optical Flow
opticalFlow = vision.OpticalFlow('Method','Horn-Schunck','ReferenceFrameDelay', 1); % ToDo:Explore the function posiblities
opticalFlow.OutputValue = 'Horizontal and vertical components in complex form';     % Sets a complex output for ploting

%% SELECT VIDEO
VIDEO='Videos/VIDEO0159.mp4';
videoReader = vision.VideoFileReader(VIDEO ,'ImageColorSpace','RGB','VideoOutputDataType','uint8');  % ToDo:'VideoOutputDataType'???
[g,mask,rs,thr,AreaOF,radius,distCrossing]=videoloader(VIDEO);

%% LOOP
%Video loop
TotalCircles=[];
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
    
    % LABELING (input:frameOFThr   output:TotalCircles
    [Labels, NUM] = bwlabel(frameOFThr); % Labeling of the moving areas
    Features = regionprops(Labels,'Area', 'BoundingBox');   % Region Features
    
    % BOUNDING BOX
    Boxes=zeros(NUM,4);
    TotalCircles1=[];
    for i=1:NUM
        Area=[Features(i,:).Area];
        if Area>=AreaOF   % If the area es less don't search for de boxes and cercles.
            Bbox=[Features(i,:).BoundingBox];
            Boxes=[Boxes; Bbox]; %Boxes per frame
            % CIRCLES
            [Circles,img]=FindCircles(Bbox,frame,radius);
            TotalCircles1=[TotalCircles1; Circles];
        end    
    end
    TotalCircles=[TotalCircles; TotalCircles1];
   
    tictoc=[tictoc; toc];
end
mean(tictoc)
