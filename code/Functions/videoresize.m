%%VIDEO RESIZE%%


%% clear
clc
clear all
close all

writerObj = VideoWriter('Videos/peaks.mp4');
open(writerObj);

%% SELECT VIDEO
VIDEO='Tomas/VIDEO0159.avi';
videoReader = vision.VideoFileReader(VIDEO ,'ImageColorSpace','RGB','VideoOutputDataType','uint8');
[g,mask,rs,thr,AreaOF,radius,distCrossing]=videoloader(VIDEO);

while ~isDone(videoReader)

    frame = step(videoReader);       % Load a frame
    frame = imresize(frame,rs);      % Resize frame
    writeVideo(writerObj,frame);
end


release(videoReader);
close(writerObj);
    
    