% This function is called by the timer to display one frame of the figure
 
function FrameRateDisplay(obj, event,vid)

% Constants Optical Flow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opticalFlow = vision.OpticalFlow('Method','Horn-Schunck',...
    'ReferenceFrameDelay', 1); % ToDo:Explore the function posiblities
opticalFlow.OutputValue = 'Horizontal and vertical components in complex form';...
    % Sets a complex output for ploting
CircleInserter = vision.ShapeInserter('Shape','Circles','BorderColor','Custom',...
    'CustomBorderColor',uint8([255 0 0]));
BoxInserter = vision.ShapeInserter('Shape','Rectangles','BorderColor','Custom',...
    'CustomBorderColor',uint8([255 0 0]));
distCrossing=50;

g = fspecial('gaussian',15,1);  % Gaussian Mask
mc = strel('disk',11);          % Closing Mask 21
md = strel('disk',5);           % Dilation Mask 3
thr = 0.2;
AreaOF = 2000;
radius = [25 40];

Position = zeros(6,800);        %Initialize the Position
Distance = zeros(800,6);        %Initialize the Distance
Velocity = zeros(800,6);        %Initialize the Velocity
Acceletarion = zeros(800,6);    %Initialize the Acceleration
k=2;                            %Initialize the loop
Direction = zeros(6,500);       %Initialize the Direction
theta = 0 : 0.05 : 2*pi;        %Angle Pixels Circle

l=[];
nc=[];
k=2;
dir=zeros(75);
DistThr=25;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


persistent IM;
persistent frame;
persistent handlesRawOF;
persistent handlesRawOFThr;
persistent handlesRawCircle;

trigger(vid);
IM=getdata(vid,1);
 
if isempty(frame) % if first execution, we create the figure objects
   %% Plot Optical Flow Image (mag)
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   frame = mat2gray(IM);
   S=size(frame);
   % Optical Flow
   frameOF = step(opticalFlow, frame);
   frameOF = imfilter(frameOF,g);
   % Morphological
   mag = abs(frameOF);
   subplot(2,3,2);
   handlesRawOF=imshow(mag);
   title('Optical Flow Magnitud');
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   
   %% Plot videoOPThreshold (OFThr)
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   OFThr = im2bw(mag, thr);            % Threshold
   OFThr = imclose(OFThr,mc);          % Closing
   OFThr = imfill(OFThr,'holes');      % Fill
   OFThr = imopen(OFThr,mc);           % Opening
   
   % MOVING ZONES LABELING
   frame = repmat(frame,[1 1 3]);      
    
   % LABELING (input:frameOFThr   output:TotalCircles
   [Labels, NUM] = bwlabel(OFThr); % Labeling of the moving areas
   Features = regionprops(Labels,'Area', 'BoundingBox');   % Region Features
   
   % BOUNDING BOX
   OFThr = repmat(double(OFThr),[1 1 3]);
   subplot(2,3,3);
   handlesRawOFThr=imshow(OFThr);
   title('Optical Flow Threshold');
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
                
    subplot(2,3,1)
    out2 = step(CircleInserter, frame, TotalCircles);
    handlesRawCircle = imshow(out2);
    title('Image with Bounding Circles')
        
       end
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
else
   %% We only update what is needed
      
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   frame = mat2gray(IM);
   % Optical Flow
   frameOF = step(opticalFlow, frame);
   frameOF = imfilter(frameOF,g);
   % Morphological
   mag = abs(frameOF);
   set(handlesRawOF,'CData',mag);
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   
   OFThr = im2bw(mag, thr);            % Threshold
   OFThr=imclose(OFThr,mc);            % Closing
   OFThr = imfill(OFThr,'holes');      % Fill%%%%%%%%%%%%
   OFThr=imopen(OFThr,mc);             % Opening
   % MOVING ZONES LABELING
   frame = repmat(frame,[1 1 3]);      
    
   % LABELING (input:frameOFThr   output:TotalCircles
   [Labels, NUM] = bwlabel(OFThr); % Labeling of the moving areas
   Features = regionprops(Labels,'Area', 'BoundingBox');   % Region Features
   
   % BOUNDING BOX
   OFThr = repmat(double(OFThr),[1 1 3]);
   set(handlesRawOFThr,'CData',OFThr);
     
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
   
    out2 = step(CircleInserter, frame, TotalCircles);
    set(handlesRawCircle,'CData',out2);
    end
    

   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
