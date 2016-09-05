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
    
%     [Labels, NUM] = bwlabel(OFThr); % Labeling of the moving areas
%     Features = regionprops(Labels,'Area', 'BoundingBox');   % Region Features

%     Boxes=[];
    Image=OFThr.*frame;
    frameCircle = repmat(frame,[1 1 3]);      % Frame to RGB for plot the red circles
   
%     TotalCircles=[];
%     NumCir=0;
%     for i=1:length(Features)
%         Area=[Features(i,:).Area];
%         if Area>=AreaOF   % If the area es less don't search for de boxes and cercles.
%             Bbox=[Features(i,:).BoundingBox];
%             Boxes=[Boxes; Bbox]; %Boxes per frame
%             % CIRCLES
%             [Circles,img]=FindCircles(Bbox,frame,radius);
%             TotalCircles=[TotalCircles; Circles];
%         end    
%     end

    %%%
    Circles=[];
    Ibox = Image;% Pieze of the original image
    Ibox = edge(Ibox,'sobel');% Edge detector to get more accuracy in the following function
    
    [centers radii] = imfindcircles(Ibox,radius,'Sensitivity',0.9); % Find circles function
    if ~isempty(centers)
        TotalCircles=[centers(:,1),centers(:,2),radii];
    end
    %%%
    
    
    NumCir=size(TotalCircles,1);
    % POSITION AND DIRECTION
    if ~isempty(TotalCircles)
        Position(1:NumCir,2*k-1:2*k) = TotalCircles(:,1:2);
        for i=1:NumCir
            pixelsX = ceil((3*TotalCircles(i,3)/4:TotalCircles(i,3))' * cos(theta) + Position(i,2*k-1));
                pixelsX = reshape(pixelsX,1,[]);
            pixelsY = ceil((3*TotalCircles(i,3)/4:TotalCircles(i,3))' *sin(theta)+Position(i,2*k));
                pixelsY = reshape(pixelsY,1,[]);
            sof=size(of);
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
            dir=of(sub2ind(size(of),pixelsY,pixelsX));
            Direction(i,2*k-1) = mean(dir);
        end
        %%COMENTANT RETALLAM 0.05 DE 0.028
        quiver(TotalCircles(:,1),-TotalCircles(:,2),real(Direction(1:NumCir,2*k-1)),-imag(Direction(1:NumCir,2*k-1)),0.05,'color',[0 0 1]);hold on;
        xlim([0 384]);ylim([-216 0 ]);title('Position and Velocity'); %

        out2 = step(CircleInserter, frameCircle, TotalCircles);
        step(videoVector2, out2);
    end  
    
    k=1+k;
    
    
     %% VIDEO
     

    step(videoMagnitud, magagnitude);
    step(videoOPThreshold, OFThr);

%     release(videoBbox);
%     step(videoBbox, img)  
tictoc=[tictoc; toc];
end
mean(tictoc)


%% Trajectory
DC=50;

[Traj Orien]=MTG(Position,Direction,DC);
close all
ArowSize=0.5;
NumTraj=size(Traj,3);

%Descartar Trajectorias por ruido
for i=1:NumTraj
   if nnz(Orien(:,:,i))<=10
       Orien(:,:,i)=[];
       Traj(:,:,i)=[];
   end
end
NumTraj=size(Traj,3);
Traj(1,:,:)=[];
Orien(1,:,:)=[];

%Plot quiver traj por separado
for i=1:NumTraj
   quiver(Traj(:,1,i),-Traj(:,2,i),real(Orien(:,:,i)),-imag(Orien(:,:,i)),ArowSize);hold on;
   xlim([0 384]);ylim([-216 0 ]);title('Position and Velocity'); %
%    pause;
end


figure; axis equal;
subplot(1,2,1);
hold on;



%Plot trajectorias
for i=1:NumTraj
    
    plot3(Traj(:,1,i),-Traj(:,2,i),abs(Orien(:,1,i)));
    xlim([0 384]);ylim([-216 0 ]);title('Trajectory'); %
   legendInfo{i} = ['Trajectory' num2str(i)];
end
legend(legendInfo)
%Plot velocidades
for i=1:NumTraj
    subplot(NumTraj,2,2*i);
    plot(abs(Orien(:,1,i)));
    title(['Velocity Trajectory ' num2str(i)]); %
end


%% CLOSE VIDEO
release(videoVector1);
release(videoVector2);
release(videoMagnitud);
release(videoOPThreshold);

% release(videoBbox);

release(videoReader);