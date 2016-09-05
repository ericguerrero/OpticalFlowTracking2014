function varargout = Interface(varargin)

% INTERFACE MATLAB code for Interface.fig
%      INTERFACE, by itsevbvlf, creates a new INTERFACE or raises the existing
%      singleton*.
%
%      H = INTERFACE returns the handle to a new INTERFACE or the handle to
%      the existing singleton*.
%
%      INTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERFACE.M with the given input arguments.
%
%      INTERFACE('Property','Value',...) creates a new INTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Interface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Interface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Interface

% Last Modified by GUIDE v2.5 12-Dec-2014 12:12:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Interface_OpeningFcn, ...
                   'gui_OutputFcn',  @Interface_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Interface is made visible.
function Interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Interface (see VARARGIN)
% Choose default command line output for Interface
handles.output = hObject;

global VIDEO
VIDEO = 'Videos/VIDEO0156.mp4';

axes(handles.axes6);
bg = imread('Videos/LOGO.jpg');
image(bg);
axis off; 

% Update handles structure
guidata(hObject, handles);





% UIWAIT makes Interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Interface_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes when selected object is changed in uipanel1.
function uipanel1_SelectionChangeFcn(hObject, eventdata, handles)

A=get(hObject,'String');
global VIDEO
switch A
    case '1 Ball'
    VIDEO = 'Videos/VIDEO0156.mp4';
    case '2 Balls, Fast Cros'
    VIDEO = 'Videos/VIDEO0166.mp4';
    case '3 Balls, 2 Mov'
    VIDEO = 'Videos/VIDEO0158.mp4';
    case '3 Balls, 2 Movements'
    VIDEO = 'Videos/VIDEO0159.mp4';
    case '3 Balls, 3 Movements'
    VIDEO = 'Videos/VIDEO0185.mp4';
    case '2 Balls, Direc'
    VIDEO = 'Videos/VIDEO0168.mp4';
    case '2 Balls, Direction'
    VIDEO = 'Videos/VIDEO0184.mp4';
    case '2 Balls, Faster Crossing'
    VIDEO = 'Videos/VIDEO0170.mp4';
    case '2 Balls, Jump Crossing'
    VIDEO = 'Videos/VIDEO0172D.mp4';
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

MainCode
