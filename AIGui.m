function varargout = AIGui(varargin)
% AIGUI MATLAB code for AIGui.fig
%      AIGUI, by itself, creates a new AIGUI or raises the existing
%      singleton*.
%
%      H = AIGUI returns the handle to a new AIGUI or the handle to
%      the existing singleton*.
%
%      AIGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in AIGUI.M with the given input arguments.
%
%      AIGUI('Property','Value',...) creates a new AIGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AIGui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AIGui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AIGui

% Last Modified by GUIDE v2.5 22-Jul-2015 08:39:10

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @AIGui_OpeningFcn, ...
                   'gui_OutputFcn',  @AIGui_OutputFcn, ...
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


% --- Executes just before AIGui is made visible.
function AIGui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AIGui (see VARARGIN)

% Choose default command line output for AIGui
handles.output = hObject;

%%%%%%%%%%%%%%%%%%%%%%%% Default Settings %%%%%%%%%%%%%%%%%%%%%%%%%
defaultRemoteIP = '129.78.137.229'; %IP address of Xenics computer
defaultRemotePort = 9090;
defaultLocalPort = 9091;

xenicsType = 320; %Set to 320 or 640, accordingly.

expTime = 0.001;     % Exposure time
emGain = 0;          % EM gain
HSSpeed = 0;         % Horizontal shift speed (index)
VSSpeed = 4;         % Vertical shift speed (index)
VSVolts = 0;         % Vertical shift voltage (index)

kinCycleTime = 0;    % Kinetic Cycle Time (0 sets it to minimum possible)
%numKinetics = 1000;   % Numer of frames in kinetic series
triggerMode = 0;     % 0 = internal, 1 = external
ftMode = 1;     % 0 = NFT, 1 = FT
delayTime = 0; % A wait after MEMS move before the next frame acquired

coolingOn = 1;        % =1 for On.
targetTemp = -65;     % In degrees C
showVideoOn = 1;       % =1 for On.

guiUpdateRate = 2;     % Default frame rate to update GUI 
vidUpdateRate = 5;     % Default frame rate to display live video (FPS).
                       % Actual frame rate of *camera* controlled by
                       % Kinetic Cycle Time

%lineProfYMax = 15000;  % Default max y axis for line profile


memsDefaultStepSize = 0.05;
memsScanRange = -3:0.05:3; %TODO - enter this through GUI
memsScanRange = -3:0.5:3; %TODO - enter this through GUI

MirrorSerialNumberString = 'FSC37-02-01-0310';
DriverSerialNumberString = '11140003';
HardwareDisableFlag = false;

pGain = 0.5;
iGain = 0.0;
dGain = 0.0;

% Relationship of MEMS to WFS
% memsTheta = (-59) /180*pi;
% mX = 2.8668;
% bX = 17.9658;
% mY = 2.8349;
% bY = -4.3208;

memsTheta = (30) /180*pi;
mX = -2.8348;
mY = 2.8673;
bX = 4.6337;
bY = 17.8877;

% 16 px window version:
% mX = 2.7506;
% bX = 10.0892;
% mY = 2.8174;
% bY = -4.5733;

hardLims = [ [-3, 3] ; [-3, 3] ]; %MEMS will not move past this in loop
segNums = [30, 27, 25, 23, 21, 37, 35, 17]; %Segment numbers
winSize = 24; %Size of subwindow for COG measurement
winCentFilename = 'winCents.mat';
bgRegion = [ 55,75 ; 55, 75]; %row;col
nSubs = length(segNums);

%For now, set the target positions to the original centres
targetPosns = zeros(8,2);

bgHeight = bgRegion(1,2) - bgRegion(1,1);
bgWidth = bgRegion(2,2) - bgRegion(2,1);
bgRect = [bgRegion(2,1) bgRegion(1,1) bgWidth bgHeight];

%%%%%%%%% Put variables into application data %%%%%%%%%
setappdata(handles.AIGui,'expTime',expTime)
setappdata(handles.AIGui,'emGain',emGain)
setappdata(handles.AIGui,'HSSpeed',HSSpeed)
setappdata(handles.AIGui,'VSSpeed',VSSpeed)
setappdata(handles.AIGui,'VSVolts',VSVolts)
setappdata(handles.AIGui,'kinCycleTime',kinCycleTime)
%setappdata(handles.AIGui,'numKinetics',numKinetics)
setappdata(handles.AIGui,'triggerMode',triggerMode)
setappdata(handles.AIGui,'ftMode',ftMode)
setappdata(handles.AIGui,'delayTime',delayTime)
setappdata(handles.AIGui,'targetTemp',targetTemp)
setappdata(handles.AIGui,'coolingOn',coolingOn)
setappdata(handles.AIGui,'guiUpdateRate',guiUpdateRate)
setappdata(handles.AIGui,'showVideoOn',showVideoOn)
setappdata(handles.AIGui,'runState',0)
setappdata(handles.AIGui,'xenicsType',xenicsType)
setappdata(handles.AIGui,'pGain',pGain)
setappdata(handles.AIGui,'iGain',iGain)
setappdata(handles.AIGui,'dGain',dGain)
setappdata(handles.AIGui,'aiLoopIsClosed',false)
setappdata(handles.AIGui,'memsWfsCoeffs',[mX bX mY bY])
setappdata(handles.AIGui,'memsWfsRmat', ...
    [cos(memsTheta), -sin(memsTheta) ; sin(memsTheta), cos(memsTheta)])
setappdata(handles.AIGui,'segNums',segNums)
setappdata(handles.AIGui,'winSize',winSize)
load(winCentFilename)
setappdata(handles.AIGui,'winCents',winCents)
setappdata(handles.AIGui,'bgVal', 0.);
setappdata(handles.AIGui,'bgRegion', bgRegion);
setappdata(handles.AIGui,'bgRect', bgRect);
setappdata(handles.AIGui,'targetPosns',targetPosns)
setappdata(handles.AIGui,'memsScanRange',memsScanRange);
setappdata(handles.AIGui,'curPx',zeros(nSubs,2));
setappdata(handles.AIGui,'maxVal', 0.);
setappdata(handles.AIGui,'hardLims', hardLims);
allSubIms = zeros(winSize, winSize, nSubs);
setappdata(handles.AIGui,'allSubIms', allSubIms);


%%%%%%%%%%%%%%%%%%% Set GUI values %%%%%%%%%%%%%%%%%%%
set(handles.expTimeBox,'string',num2str(expTime))
set(handles.emGainBox,'string',num2str(emGain))
set(handles.hsSpeedBox,'string',num2str(HSSpeed))
set(handles.vsSpeedBox,'string',num2str(VSSpeed))
set(handles.vsAmpBox,'string',num2str(VSVolts))
set(handles.cycTimeBox,'string',num2str(kinCycleTime))
%set(handles.numFramesBox,'string',num2str(numKinetics))
set(handles.delayTimeBox,'string',num2str(delayTime))
set(handles.tempSetBox,'string',num2str(targetTemp))

%set(handles.axesDetectorView,'XTickLabel','')
%set(handles.axesDetectorView,'YTickLabel','')

set(handles.camStateText,'String','Off');
set(handles.camStateText,'ForegroundColor',[1 0 0]);

set(handles.stepSizeBox,'String',num2str(memsDefaultStepSize));
set(handles.memsPosnTable,'Data',zeros(37,3));

set(handles.remoteIPBox,'String',defaultRemoteIP);
set(handles.remotePortBox,'String',num2str(defaultRemotePort));
set(handles.localPortBox,'String',num2str(defaultLocalPort));

set(handles.pGainBox,'String',num2str(pGain));
set(handles.iGainBox,'String',num2str(iGain));
set(handles.dGainBox,'String',num2str(dGain));
set(handles.loopStatusText,'String','Open');
set(handles.loopStatusText,'ForegroundColor',[1 0 0]);


if ftMode == 1
    set(handles.NFTBtn,'Value',0);
    set(handles.FTBtn,'Value',1);
else
    set(handles.NFTBtn,'Value',1);
    set(handles.FTBtn,'Value',0);
end

if coolingOn == 1
    set(handles.ccOnBtn,'Value',1);
    set(handles.ccOffBtn,'Value',0);
else
    set(handles.ccOnBtn,'Value',0);
    set(handles.ccOffBtn,'Value',1);
end

if showVideoOn == 1
    set(handles.showVideoCheckbox,'Value',1);
else
    set(handles.showVideoCheckbox,'Value',0);
end


%%%%%%%%%% Turn things on %%%%%%%%%%
disp('Loading Andor library...');
header='C:\Program Files\Andor iXon\Drivers\ATMCD32D.H';
[notfound, warnings] = loadlibrary('C:\Program Files\Andor iXon\Drivers\atmcd32d.dll', header);


% Connect to MEMS and turn it on
disp('Initaliasing MEMS...')
memsHandle = MirrorConnect(MirrorSerialNumberString, ...
    DriverSerialNumberString, HardwareDisableFlag);
if HardwareDisableFlag
    disp('MEMS hardware-disable flag is set!')
end
numMemsSegs=37;
PTTPositionFlat = zeros(numMemsSegs,3);
setappdata(handles.AIGui,'PTTPositionFlat',PTTPositionFlat);
memsSegsList=1:numMemsSegs;
SetMirrorPosition(memsHandle, memsSegsList, PTTPositionFlat); %Flatten MEMS
MirrorSendSettings(memsHandle);
setappdata(handles.AIGui,'memsHandle',memsHandle)
setappdata(handles.AIGui,'memsSegsList',memsSegsList)
PTTPositionOn = zeros(numMemsSegs,3);
setappdata(handles.AIGui,'PTTPositionOn',PTTPositionOn);
PTTPositionOff = zeros(numMemsSegs,3);
setappdata(handles.AIGui,'PTTPositionOff',PTTPositionOff);
PTTCurrentPosition = zeros(numMemsSegs,3);
setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);





% Initalise camera
disp('Initaliasing Camera...')
error=calllib('atmcd32d','Initialize','C:\Program Files\Andor iXon\Drivers');

if error == 20002
    disp('Initialisation Succesful')
else
    disp('Initialisation Error')
end

%getCameraModes;

% Set shutter to open
error=calllib('atmcd32d','SetShutter',1,1,50,50);

% Set read mode to Image
error=calllib('atmcd32d','SetReadMode',4);

% Set acquisition mode to Run til abort
error=calllib('atmcd32d','SetAcquisitionMode',5);
error=calllib('atmcd32d','SetExposureTime',expTime);

% Set frame transfer mode
error=calllib('atmcd32d','SetFrameTransferMode',ftMode);

% Set trigger mode to internal
error=calllib('atmcd32d','SetTriggerMode',0);

error=calllib('atmcd32d','SetKineticCycleTime',kinCycleTime);
error=calllib('atmcd32d','SetEMGainMode',2); % Believed to be 'real', docs wrong.
error=calllib('atmcd32d','SetEMAdvanced',0); %%%% Set to 1 at your peril!!! (RTFM) %%%%
error=calllib('atmcd32d','SetEMCCDGain',emGain);
error=calllib('atmcd32d','SetHSSpeed',0,HSSpeed); %Also sets to EM output amp
error=calllib('atmcd32d','SetVSSpeed',VSSpeed);
error=calllib('atmcd32d','SetVSAmplitude',VSVolts);

% Cooling
error=calllib('atmcd32d','SetFanMode',0); %0 is full on.
error=calllib('atmcd32d','SetTemperature',targetTemp);
if coolingOn == 1
    error=calllib('atmcd32d','CoolerON');
else
    error=calllib('atmcd32d','CoolerOFF');
end

% Setup image size
imw=libpointer('int32Ptr',0);
imh=libpointer('int32Ptr',0);
error=calllib('atmcd32d','GetDetector',imw,imh);
imwidth=imw.value;
imheight=imw.value;
npixels=imwidth*imheight;
imPtr = libpointer('int32Ptr',zeros(imwidth,imheight));
error=calllib('atmcd32d','SetImage',1,1,1,imwidth,1,imheight);
setappdata(handles.AIGui,'imPtr',imPtr);
setappdata(handles.AIGui,'npixels',npixels);
setappdata(handles.AIGui,'currentIm',zeros(imwidth,imheight));
setappdata(handles.AIGui,'imwidth',imwidth);

%auxFig = figure(1);
%auxFig = gca;
%setappdata(handles.AIGui,'auxFig',auxFig);

% Setup timers
handles.vidTimer = timer(...
    'ExecutionMode', 'fixedRate', ...
    'Period', 1/vidUpdateRate, ...
    'TimerFcn', {@updateVidFn,hObject} );

handles.valTimer = timer(...
    'ExecutionMode', 'fixedRate', ...
    'Period', 1/guiUpdateRate, ...
    'TimerFcn', {@updateGuiFn,hObject} );

% Update handles structure
guidata(hObject, handles);

start(handles.valTimer);
if showVideoOn == 1
    start(handles.vidTimer);
end


% UIWAIT makes AIGui wait for user response (see UIRESUME)
% uiwait(handles.AIGui);


% --- Outputs from this function are returned to the command line.
function varargout = AIGui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function getCameraModes(handles)
ptr=libpointer('int32Ptr',0);
fptr=libpointer('singlePtr',0);

disp('HS Speeds:');
error=calllib('atmcd32d','GetNumberHSSpeeds',0,0,ptr);
nHSSpeeds = ptr.value;
for k = 0:nHSSpeeds-1
    error=calllib('atmcd32d','GetHSSpeed',0,0,k,fptr);
    disp([double(k) fptr.value]);
end

disp('VS Speeds:');
error=calllib('atmcd32d','GetNumberVSSpeeds',ptr);
nVSSpeeds = ptr.value;
for k = 0:nVSSpeeds-1
    error=calllib('atmcd32d','GetVSSpeed',k,fptr);
    disp([double(k) fptr.value]);
end

disp('VS Voltages:');
error=calllib('atmcd32d','GetNumberVSAmplitudes',ptr);
nVSAmps = ptr.value
%sptr=libpointer('stringPtrPtr',{''});
% for k = 0:nVSAmps-1
%     error=calllib('atmcd32d','GetVSAmplitudeValue',k,ptr);
%     disp([double(k) ptr.value]);
%     %disp(k);
%     %disp(sptr.value);
% end

disp('Preamp gains:');
error=calllib('atmcd32d','GetNumberPreAmpGains',ptr);
nPreampGains = ptr.value
for k = 0:nPreampGains-1
    error=calllib('atmcd32d','GetPreAmpGain',k,fptr);
    disp([double(k) fptr.value]);
end


function expTimeBox_Callback(hObject, eventdata, handles)
% hObject    handle to expTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of expTimeBox as text
%        str2double(get(hObject,'String')) returns contents of expTimeBox as a double
expTime=str2double(get(hObject,'String'));
setappdata(handles.AIGui,'ExpTime',expTime)
calllib('atmcd32d','SetExposureTime',expTime);

% --- Executes during object creation, after setting all properties.
function expTimeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to expTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function emGainBox_Callback(hObject, eventdata, handles)
% hObject    handle to emGainBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

emGain=str2double(get(hObject,'String'));
setappdata(handles.AIGui,'emGain',emGain)
calllib('atmcd32d','SetEMCCDGain',emGain);

% --- Executes during object creation, after setting all properties.
function emGainBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to emGainBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function hsSpeedBox_Callback(hObject, eventdata, handles)
% hObject    handle to hsSpeedBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

hsSpeed=str2double(get(hObject,'String'));
setappdata(handles.AIGui,'hsSpeed',hsSpeed)
calllib('atmcd32d','SetHSSpeed',0,hsSpeed);

% --- Executes during object creation, after setting all properties.
function hsSpeedBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hsSpeedBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function vsSpeedBox_Callback(hObject, eventdata, handles)
% hObject    handle to vsSpeedBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

vsSpeed=str2double(get(hObject,'String'));
setappdata(handles.AIGui,'vsSpeed',vsSpeed)
calllib('atmcd32d','SetVSSpeed',vsSpeed);

% --- Executes during object creation, after setting all properties.
function vsSpeedBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vsSpeedBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function vsAmpBox_Callback(hObject, eventdata, handles)
% hObject    handle to vsAmpBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

vsAmp=str2double(get(hObject,'String'));
setappdata(handles.AIGui,'vsAmp',vsAmp)
calllib('atmcd32d','SetVSAmplitude',vsAmp);

% --- Executes during object creation, after setting all properties.
function vsAmpBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vsAmpBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function cycTimeBox_Callback(hObject, eventdata, handles)
% hObject    handle to cycTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

kinCycleTime=str2double(get(hObject,'String'));
setappdata(handles.AIGui,'kinCycleTime',kinCycleTime)
calllib('atmcd32d','SetKineticCycleTime',kinCycleTime);

%Print the actual cycle time
actualExp=libpointer('singlePtr',0);
actualAcc=libpointer('singlePtr',0);
actualCyc=libpointer('singlePtr',0);
error=calllib('atmcd32d','GetAcquisitionTimings',actualExp,actualAcc,actualCyc);
disp(strcat('Actual cycle time: ',num2str(actualCyc.value)))

% --- Executes during object creation, after setting all properties.
function cycTimeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cycTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function tempSetBox_Callback(hObject, eventdata, handles)
% hObject    handle to tempSetBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

targetTemp=str2double(get(hObject,'String'));
setappdata(handles.AIGui,'targetTemp',targetTemp)
error=calllib('atmcd32d','SetTemperature',targetTemp);

% --- Executes during object creation, after setting all properties.
function tempSetBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tempSetBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in runBtn.
function runBtn_Callback(hObject, eventdata, handles)
% hObject    handle to runBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if getappdata(handles.AIGui,'runState') == 0
    setappdata(handles.AIGui,'runState',1)
    set(handles.camStateText,'String','Running');
    set(handles.camStateText,'ForegroundColor',[0 1 0]);
    greyOutCamControls(handles, true)
    mainRunLoop(handles);
else
    setappdata(handles.AIGui,'runState',0)
    set(handles.camStateText,'String','Off');
    set(handles.camStateText,'ForegroundColor',[1 0 0]);
    greyOutCamControls(handles, false)
end


%%%% This is the main running loop %%%%
function mainRunLoop(handles)

nSamps = 200; %Number of loops to average for timing
nIts = 15000; %Number of measurements to save to a file

imPtr = getappdata(handles.AIGui,'imPtr');
npixels = getappdata(handles.AIGui,'npixels');
segNums = getappdata(handles.AIGui,'segNums');
memsWfsRmat = getappdata(handles.AIGui,'memsWfsRmat');
cf = getappdata(handles.AIGui,'memsWfsCoeffs');
imwidth = getappdata(handles.AIGui,'imwidth');
hardLims = getappdata(handles.AIGui,'hardLims');
auxFig = getappdata(handles.AIGui,'auxFig');
mX = cf(1);
bX = cf(2);
mY = cf(3);
bY = cf(4);
nSubs = length(segNums);
winSize = getappdata(handles.AIGui,'winSize');
winCents = getappdata(handles.AIGui,'winCents');
bgVal = getappdata(handles.AIGui,'bgVal'); %Median/mean background level
bgRegion = getappdata(handles.AIGui,'bgRegion');
Rmat = getappdata(handles.AIGui,'memsWfsRmat');
allCogX = zeros(nSubs,1);
allCogY = zeros(nSubs,1);
allCogXs = zeros(nSubs,nIts);
allCogYs = zeros(nSubs,nIts);
imageCube = zeros(imwidth,imwidth);
newMemsPosnX = zeros(nSubs,1);
newMemsPosnY = zeros(nSubs,1);
%allSubIms = zeros(winSize, winSize, nSubs);
allSubImsMulti = zeros(winSize, winSize, nSubs, nIts); %For saving
errorX=zeros(nSubs,1);
errorY=zeros(nSubs,1);
curPx=zeros(nSubs,2);
inds=1:winSize;
setappdata(handles.AIGui,'saveDataState',false)
count=1;

% This should be the optimum position for a flat wavefront
PTTPositionOn=getappdata(handles.AIGui,'PTTPositionOn');

memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
targetPosns=getappdata(handles.AIGui,'targetPosns');
segInds = memsSegsList(segNums);

% Put the actual current positions into curMemsPosnX,Y
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
curMemsPosnX = ReachablePositions(segInds,2);
curMemsPosnY = ReachablePositions(segInds,3);
curAllMemsPosn = ReachablePositions;

% TODO - Haven't checked this works yet
% if bgVal < 0
%     errordlg('Background not defined','You need to measure the background first.');
%     return
% end

curIt = 0;

%%%% Testing
%imStoreCube=zeros(128,128,1000);
%curIt2 = 1;
%%%%

calllib('atmcd32d','StartAcquisition');
tic
while getappdata(handles.AIGui,'runState') == 1
    
    % If you don't do this then you can never leave the loop...
    drawnow limitrate
    
    % Put the actual current positions into curMemsPosnX,Y
    [ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
    curMemsPosnX = ReachablePositions(segInds,2);
    curMemsPosnY = ReachablePositions(segInds,3);
    curAllMemsPosn = ReachablePositions;

    error=calllib('atmcd32d','WaitForAcquisition');
    getimerr=calllib('atmcd32d','GetMostRecentImage',imPtr,npixels);
    im=imPtr.value;
    setappdata(handles.AIGui,'currentIm',im);
    
    % Measure background
    bgImage = im(bgRegion(1,1):bgRegion(1,2),bgRegion(2,1):bgRegion(2,2));
    bgVal = mean(bgImage(:));
    setappdata(handles.AIGui,'bgVal', bgVal);
    maxVal = max(im(:));
    setappdata(handles.AIGui,'maxVal', maxVal);
    
    if curIt == nSamps
        % Update loop speed value
        tm=toc;
        speed = 1/(tm/nSamps);
        setappdata(handles.AIGui,'loopSpeed',speed);
        %disp(['Speed: ' num2str(speed)])
        
        % Print the most recent error values, to get an idea of performance
        %disp(['Mean error X: ' num2str(mean(errorX))])
        %disp(['Mean error Y: ' num2str(mean(errorY))])
        %disp('')
        %allCogX
        %allCogY
        
        %Update the MEMS table
        [ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
        set(handles.memsPosnTable,'Data',ReachablePositions);
        
        %%%%%%%% TESTING - Show subims in another window
        %imagesc(im,'Parent',auxFig)
        %subplot(4,2,1,auxFig,'Parent','auxFig')
        %hold(auxFig,'on')
%         for kk = 1:nSubs
%             a=subplot(4,2,kk,auxFig);
%             imagesc(allSubIms(:,:,kk))%,'Parent',a)%,'Parent',auxFig)
%             %axis equal
%         end
        %hold(auxFig,'off')
        %%%%%%%%
        
        curIt = 0;
        tic
    end
    curIt = curIt+1;
    
    %%% Testing
%     imStoreCube(:,:,curIt2) = im;
%     if curIt2 == 1000
%         break
%     end
%     curIt2 = curIt2+1;
    %%%
    
    %pause(0.01)
    
    %%%%%%%%%% Actual AI stuff %%%%%%%%%%
    allSubIms = zeros(winSize, winSize, nSubs);
    imBgSubt = im - bgVal;
    doCorr = getappdata(handles.AIGui,'aiLoopIsClosed');
    pGain = getappdata(handles.AIGui,'pGain');
    iGain = getappdata(handles.AIGui,'iGain');
    dGain = getappdata(handles.AIGui,'dGain');
    
    % Loops through subims, but can maybe do this faster with some array op
    for kk = 1:nSubs
        allSubIms(:,:,kk) = imBgSubt(winCents(kk,1)-winSize/2:winCents(kk,1)+winSize/2-1, ...
            winCents(kk,2)-winSize/2:winCents(kk,2)+winSize/2-1);
        
        % Measure COGs
        hSum=sum(allSubIms(:,:,kk), 2);
        vSum=sum(allSubIms(:,:,kk), 1);
        cur = [sum(vSum.*inds)/sum(vSum) ; sum(hSum'.*inds)/sum(hSum)];
        curPx(kk,:) = cur;
        cur = Rmat*cur;
        cur(1) = (cur(1) - bX)/mX;
        cur(2) = (cur(2) - bY)/mY;
        allCogX(kk) = cur(1);
        allCogY(kk) = cur(2);
 
        errorX(kk) = cur(1) - targetPosns(kk, 1);
        errorY(kk) = cur(2) - targetPosns(kk, 2);

        % Do correction
        if doCorr
            newMemsPosnX(kk) = curMemsPosnX(kk) - errorX(kk)*pGain;
            newMemsPosnY(kk) = curMemsPosnY(kk) - errorY(kk)*pGain;
        end
        
        % Enforce hard limits
        if (newMemsPosnX(kk) < hardLims(1,1))
            newMemsPosnX(kk) = hardLims(1,1);
        end
        if (newMemsPosnX(kk) > hardLims(1,2))
            newMemsPosnX(kk) = hardLims(1,2);
        end
        if (newMemsPosnY(kk) < hardLims(2,1))
            newMemsPosnY(kk) = hardLims(2,1);
        end
        if (newMemsPosnY(kk) > hardLims(2,2))
            newMemsPosnY(kk) = hardLims(2,2);
        end

    end
    setappdata(handles.AIGui,'curPx',curPx);
    setappdata(handles.AIGui,'allSubIms',allSubIms);

    if doCorr
        % Send new positions to MEMS
        curAllMemsPosn(segInds,2) = newMemsPosnX;
        curAllMemsPosn(segInds,3) = newMemsPosnY;    
        SetMirrorPosition(memsHandle, memsSegsList, curAllMemsPosn);
        MirrorSendSettings(memsHandle);
        %%%%
    
        curMemsPosnX = newMemsPosnX;
        curMemsPosnY = newMemsPosnY;   
    end
     
    
    if getappdata(handles.AIGui,'saveDataState')
        allCogXs(:,count) = allCogX;
        allCogYs(:,count) = allCogY;
        %imageCube(:,:,count) = im; %%%%%%%%%%%%%%%%%%%%%%%%%%%
        allSubImsMulti(:,:,:,count) = allSubIms;
        count=count+1;
        if count == nIts+1
            setappdata(handles.AIGui,'saveDataState',false)
            set(handles.saveDataBtn,'ForegroundColor',[0 0 1]);
            drawnow
            count=1;
            dataFilename=['data\aiDataSave_' datestr(datetime,30)];
            save(dataFilename,'allCogXs','allCogYs','allSubImsMulti')
            set(handles.saveDataBtn,'ForegroundColor',[0 0 0]);
            
            disp('SD COG for X and Y:')
            disp(std(allCogXs,0,2))
            disp(std(allCogYs,0,2))
        end
    end
    
    % TODO - Put stuff in shared memory:
    % im, allSubIms, allCogX, allCogY, newMemsPosnX, newMemsPosnY, curMemsPosnX,
    % curMemsPosny
     
    
    delayTime = str2double(get(handles.delayTimeBox,'String'));
    if delayTime > 0
        tstart=tic;
        while toc(tstart) < delayTime
            % Do nothing.
            % Note pause() doesn't seem to work with very short waits,
            % so using this method instead.
        end
    end
    %pause(1e-16)%%%%%%%%%%%%%%
end

calllib('atmcd32d','AbortAcquisition');

setappdata(handles.AIGui,'loopSpeed',0);
setappdata(handles.AIGui,'runState',0)
set(handles.camStateText,'String','Off');
set(handles.camStateText,'ForegroundColor',[1 0 0]);   

% Find current positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;
setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);


% --- Executes on button press in showVideoCheckbox.
function showVideoCheckbox_Callback(hObject, eventdata, handles)
% hObject    handle to showVideoCheckbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of showVideoCheckbox
showVideoOn = get(hObject,'Value');
setappdata(handles.AIGui,'showVideoOn',showVideoOn)
if showVideoOn == 1
    start(handles.vidTimer);
else
    stop(handles.vidTimer);
end


% --- Executes on button press in exitBtn.
function exitBtn_Callback(hObject, eventdata, handles)
% hObject    handle to exitBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

disp('Exiting...')

stop(handles.valTimer)
stop(handles.vidTimer)

setappdata(handles.AIGui,'runState',0)
% Close shutter
error=calllib('atmcd32d','SetShutter',0,2,10,10);
pause(2)
error=calllib('atmcd32d','ShutDown');

%pause(2)
%unloadlibrary atmcd32d;
%pause(1)
disp('Skipping unload library')


% Release MEMS
memsHandle = getappdata(handles.AIGui,'memsHandle');
MirrorRelease(memsHandle)
pause(1)

delete(handles.AIGui);
delete(handles.valTimer)
delete(handles.vidTimer)
disp('Exited nicely. Thankyou.');


function updateGuiFn(hObject,eventdata,hfigure)
handles = guidata(hfigure);

%Update camera temperature
if getappdata(handles.AIGui,'runState') == 0
    curTempPtr=libpointer('int32Ptr',0);
    tempError=calllib('atmcd32d','GetTemperature',curTempPtr);
    curTemp=curTempPtr.value;
    set(handles.curTempText,'string',num2str(curTemp))
    if tempError == 20036 %stabilised
        set(handles.curTempText,'ForegroundColor',[0 1 0])
    else
        set(handles.curTempText,'ForegroundColor',[1 0 0])
    end
else
    set(handles.curTempText,'string','--')
end

%Update loop speed
loopSpeed = getappdata(handles.AIGui,'loopSpeed');
set(handles.loopSpeedText,'String',num2str(loopSpeed));

%Update image info
bgVal = getappdata(handles.AIGui,'bgVal');
set(handles.bgValText,'String',num2str(bgVal));
maxVal = getappdata(handles.AIGui,'maxVal');
set(handles.maxValText,'String',num2str(maxVal));

% Show individual subims (thsi slows things down though)
% allSubIms=getappdata(handles.AIGui,'allSubIms');
% imagesc(allSubIms(:,:,1),'Parent',handles.subAx1)
% imagesc(allSubIms(:,:,2),'Parent',handles.subAx2)
% imagesc(allSubIms(:,:,3),'Parent',handles.subAx3)
% imagesc(allSubIms(:,:,4),'Parent',handles.subAx4)
% imagesc(allSubIms(:,:,5),'Parent',handles.subAx5)
% imagesc(allSubIms(:,:,6),'Parent',handles.subAx6)
% imagesc(allSubIms(:,:,7),'Parent',handles.subAx7)
% imagesc(allSubIms(:,:,8),'Parent',handles.subAx8)


function updateVidFn(hObject,eventdata,hfigure)
handles = guidata(hfigure);
segNums = getappdata(handles.AIGui,'segNums');
bgRect = getappdata(handles.AIGui,'bgRect');
%targetPosns=getappdata(handles.AIGui,'targetPosns');

%Update detector view
im = getappdata(handles.AIGui,'currentIm');
imagesc(im,'Parent',handles.axesDetectorView)
%set(handles.axesDetectorView,'XTickLabel','')
%set(handles.axesDetectorView,'YTickLabel','')
  
hold(handles.axesDetectorView,'on')
nSubs = length(getappdata(handles.AIGui,'segNums'));
curPxGlobal = zeros(nSubs,2);
curPx = getappdata(handles.AIGui,'curPx');
winCents = getappdata(handles.AIGui,'winCents');
winSize = getappdata(handles.AIGui,'winSize');

for kk = 1:nSubs
    curPxGlobal(kk,1) = curPx(kk,1)-1 + winCents(kk,2) - winSize/2;
    curPxGlobal(kk,2) = curPx(kk,2)-1 + winCents(kk,1) - winSize/2;
end
plot(curPxGlobal(:,1),curPxGlobal(:,2),'r+','Parent',handles.axesDetectorView)

for kk = 1:nSubs
    str = ['   ' num2str(segNums(kk))];
    text(curPxGlobal(kk,1),curPxGlobal(kk,2),str, ...
        'Parent',handles.axesDetectorView,'Color','red')
end

plot(winCents(:,2),winCents(:,1),'bo','Parent',handles.axesDetectorView)
rectangle('Position',bgRect,'EdgeColor','y','Linestyle','--','Parent',handles.axesDetectorView)

hold(handles.axesDetectorView,'off')




% --- Executes when selected object is changed in CamCoolPanel.
function CamCoolPanel_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in CamCoolPanel 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch get(eventdata.NewValue,'Tag') % Get Tag of selected object.
    case 'ccOnBtn'
        calllib('atmcd32d','CoolerON');
        setappdata(handles.AIGui,'coolingOn',1);
        %logging('Camera cooling on');
    case 'ccOffBtn'
        calllib('atmcd32d','CoolerOFF');
        setappdata(handles.AIGui,'coolingOn',0);
        %logging('Camera cooling off');
end


% --- Executes when entered data in editable cell(s) in memsPosnTable.
function memsPosnTable_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to memsPosnTable (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');

% Get new values and put into matrix
PTTCurrentPosition = getappdata(handles.AIGui,'PTTCurrentPosition');
PTTCurrentPosition(eventdata.Indices(1),eventdata.Indices(2))=eventdata.NewData;

% Send new values to MEMS
SetMirrorPosition(memsHandle, memsSegsList, PTTCurrentPosition);
MirrorSendSettings(memsHandle);

% Find actual positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;

% Store new positions
setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);


% --- Executes on button press in pistnUpBtn.
function pistnUpBtn_Callback(hObject, eventdata, handles)
% hObject    handle to pistnUpBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
stepSize=str2double(get(handles.stepSizeBox,'String'));
PTTCurrentPosition = getappdata(handles.AIGui,'PTTCurrentPosition');

if segToMove == 0
    PTTCurrentPosition(:,1) = PTTCurrentPosition(:,1) + stepSize;
else
    PTTCurrentPosition(segToMove,1) = PTTCurrentPosition(segToMove,1) + stepSize;
end

% Send new values to MEMS
SetMirrorPosition(memsHandle, memsSegsList, PTTCurrentPosition);
MirrorSendSettings(memsHandle);

% Find actual positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;

% Store new positions
setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);

% --- Executes on button press in pistnDnBtn.
function pistnDnBtn_Callback(hObject, eventdata, handles)
% hObject    handle to pistnDnBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
stepSize=str2double(get(handles.stepSizeBox,'String'));
PTTCurrentPosition = getappdata(handles.AIGui,'PTTCurrentPosition');

if segToMove == 0
    PTTCurrentPosition(:,1) = PTTCurrentPosition(:,1) - stepSize;
else
    PTTCurrentPosition(segToMove,1) = PTTCurrentPosition(segToMove,1) - stepSize;
end

% Send new values to MEMS
SetMirrorPosition(memsHandle, memsSegsList, PTTCurrentPosition);
MirrorSendSettings(memsHandle);

% Find actual positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;

% Store new positions
setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);

% --- Executes on button press in tipUpBtn.
function tipUpBtn_Callback(hObject, eventdata, handles)
% hObject    handle to tipUpBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
stepSize=str2double(get(handles.stepSizeBox,'String'));
PTTCurrentPosition = getappdata(handles.AIGui,'PTTCurrentPosition');

if segToMove == 0
    PTTCurrentPosition(:,2) = PTTCurrentPosition(:,2) + stepSize;
else
    PTTCurrentPosition(segToMove,2) = PTTCurrentPosition(segToMove,2) + stepSize;
end

% Send new values to MEMS
SetMirrorPosition(memsHandle, memsSegsList, PTTCurrentPosition);
MirrorSendSettings(memsHandle);

% Find actual positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;

% Store new positions
setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);

% --- Executes on button press in tipDnBtn.
function tipDnBtn_Callback(hObject, eventdata, handles)
% hObject    handle to tipDnBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
stepSize=str2double(get(handles.stepSizeBox,'String'));
PTTCurrentPosition = getappdata(handles.AIGui,'PTTCurrentPosition');

if segToMove == 0
    PTTCurrentPosition(:,2) = PTTCurrentPosition(:,2) - stepSize;
else
    PTTCurrentPosition(segToMove,2) = PTTCurrentPosition(segToMove,2) - stepSize;
end

% Send new values to MEMS
SetMirrorPosition(memsHandle, memsSegsList, PTTCurrentPosition);
MirrorSendSettings(memsHandle);

% Find actual positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;

% Store new positions
setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);

% --- Executes on button press in tiltUpBtn.
function tiltUpBtn_Callback(hObject, eventdata, handles)
% hObject    handle to tiltUpBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
stepSize=str2double(get(handles.stepSizeBox,'String'));
PTTCurrentPosition = getappdata(handles.AIGui,'PTTCurrentPosition');

if segToMove == 0
    PTTCurrentPosition(:,3) = PTTCurrentPosition(:,3) + stepSize;
else
    PTTCurrentPosition(segToMove,3) = PTTCurrentPosition(segToMove,3) + stepSize;
end

% Send new values to MEMS
SetMirrorPosition(memsHandle, memsSegsList, PTTCurrentPosition);
MirrorSendSettings(memsHandle);

% Find actual positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;

% Store new positions
setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);

% --- Executes on button press in tiltDnBtn.
function tiltDnBtn_Callback(hObject, eventdata, handles)
% hObject    handle to tiltDnBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
stepSize=str2double(get(handles.stepSizeBox,'String'));
PTTCurrentPosition = getappdata(handles.AIGui,'PTTCurrentPosition');

if segToMove == 0
    PTTCurrentPosition(:,3) = PTTCurrentPosition(:,3) - stepSize;
else
    PTTCurrentPosition(segToMove,3) = PTTCurrentPosition(segToMove,3) - stepSize;
end

% Send new values to MEMS
SetMirrorPosition(memsHandle, memsSegsList, PTTCurrentPosition);
MirrorSendSettings(memsHandle);

% Find actual positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;

% Store new positions
setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);


function segToMoveBox_Callback(hObject, eventdata, handles)
% hObject    handle to segToMoveBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of segToMoveBox as text
%        str2double(get(hObject,'String')) returns contents of segToMoveBox as a double


% --- Executes during object creation, after setting all properties.
function segToMoveBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to segToMoveBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function stepSizeBox_Callback(hObject, eventdata, handles)
% hObject    handle to stepSizeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of stepSizeBox as text
%        str2double(get(hObject,'String')) returns contents of stepSizeBox as a double


% --- Executes during object creation, after setting all properties.
function stepSizeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stepSizeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in presetOffSetBtn.
function presetOffSetBtn_Callback(hObject, eventdata, handles)
% hObject    handle to presetOffSetBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
PTTPositionOff=getappdata(handles.AIGui,'PTTPositionOff');

[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
if segToMove == 0
    PTTPositionOff = ReachablePositions;
else
    PTTPositionOff(segToMove,:) = ReachablePositions(segToMove,:);
end

setappdata(handles.AIGui,'PTTPositionOff',PTTPositionOff);

% --- Executes on button press in presetOffRestBtn.
function presetOffRestBtn_Callback(hObject, eventdata, handles)
% hObject    handle to presetOffRestBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
PTTCurrentPosition = getappdata(handles.AIGui,'PTTCurrentPosition');
PTTPositionOff=getappdata(handles.AIGui,'PTTPositionOff');

if segToMove == 0
    PTTCurrentPosition = PTTPositionOff;
else
    PTTCurrentPosition(segToMove,:) = PTTPositionOff(segToMove,:);
end

% Send new values to MEMS
SetMirrorPosition(memsHandle, memsSegsList, PTTCurrentPosition);
MirrorSendSettings(memsHandle);

% Find actual positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;

setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);

% --- Executes on button press in presetOnSetBtn.
function presetOnSetBtn_Callback(hObject, eventdata, handles)
% hObject    handle to presetOnSetBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
PTTPositionOn=getappdata(handles.AIGui,'PTTPositionOn');

[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
if segToMove == 0
    PTTPositionOn = ReachablePositions;
else
    PTTPositionOn(segToMove,:) = ReachablePositions(segToMove,:);
end

setappdata(handles.AIGui,'PTTPositionOn',PTTPositionOn);

% --- Executes on button press in presetOnRestBtn.
function presetOnRestBtn_Callback(hObject, eventdata, handles)
% hObject    handle to presetOnRestBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
PTTCurrentPosition = getappdata(handles.AIGui,'PTTCurrentPosition');
PTTPositionOn=getappdata(handles.AIGui,'PTTPositionOn');

if segToMove == 0
    PTTCurrentPosition = PTTPositionOn;
else
    PTTCurrentPosition(segToMove,:) = PTTPositionOn(segToMove,:);
end

% Send new values to MEMS
SetMirrorPosition(memsHandle, memsSegsList, PTTCurrentPosition);
MirrorSendSettings(memsHandle);

% Find actual positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;

setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);


function presetFilenameBox_Callback(hObject, eventdata, handles)
% hObject    handle to presetFilenameBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of presetFilenameBox as text
%        str2double(get(hObject,'String')) returns contents of presetFilenameBox as a double


% --- Executes during object creation, after setting all properties.
function presetFilenameBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to presetFilenameBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in presetFileSaveBtn.
function presetFileSaveBtn_Callback(hObject, eventdata, handles)
% hObject    handle to presetFileSaveBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PTTPositionOff=getappdata(handles.AIGui,'PTTPositionOff');
PTTPositionOn=getappdata(handles.AIGui,'PTTPositionOn');
PTTPositionFlat=getappdata(handles.AIGui,'PTTPositionFlat');
fileString=get(handles.presetFilenameBox,'String');
save(fileString,'PTTPositionOff','PTTPositionOn','PTTPositionFlat');

% --- Executes on button press in presetFileRestBtn.
function presetFileRestBtn_Callback(hObject, eventdata, handles)
% hObject    handle to presetFileRestBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
fileString=get(handles.presetFilenameBox,'String');
load(fileString);
setappdata(handles.AIGui,'PTTPositionOn',PTTPositionOn);
setappdata(handles.AIGui,'PTTPositionOff',PTTPositionOff);
setappdata(handles.AIGui,'PTTPositionFlat',PTTPositionFlat);

% --- Executes on button press in presetFlatSetBtn.
function presetFlatSetBtn_Callback(hObject, eventdata, handles)
% hObject    handle to presetFlatSetBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
PTTPositionFlat=getappdata(handles.AIGui,'PTTPositionFlat');

[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
if segToMove == 0
    PTTPositionFlat = ReachablePositions;
else
    PTTPositionFlat(segToMove,:) = ReachablePositions(segToMove,:);
end

setappdata(handles.AIGui,'PTTPositionFlat',PTTPositionFlat);


% --- Executes on button press in presetFlatRestBtn.
function presetFlatRestBtn_Callback(hObject, eventdata, handles)
% hObject    handle to presetFlatRestBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
segToMove=str2double(get(handles.segToMoveBox,'String'));
PTTCurrentPosition = getappdata(handles.AIGui,'PTTCurrentPosition');
PTTPositionFlat=getappdata(handles.AIGui,'PTTPositionFlat');

if segToMove == 0
    PTTCurrentPosition = PTTPositionFlat;
else
    PTTCurrentPosition(segToMove,:) = PTTPositionFlat(segToMove,:);
end

% Send new values to MEMS
SetMirrorPosition(memsHandle, memsSegsList, PTTCurrentPosition);
MirrorSendSettings(memsHandle);

% Find actual positions and put into the table
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(memsHandle, memsSegsList);
set(handles.memsPosnTable,'Data',ReachablePositions);
PTTCurrentPosition=ReachablePositions;

setappdata(handles.AIGui,'PTTCurrentPosition',PTTCurrentPosition);



function remoteIPBox_Callback(hObject, eventdata, handles)
% hObject    handle to remoteIPBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of remoteIPBox as text
%        str2double(get(hObject,'String')) returns contents of remoteIPBox as a double


% --- Executes during object creation, after setting all properties.
function remoteIPBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to remoteIPBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function remotePortBox_Callback(hObject, eventdata, handles)
% hObject    handle to remotePortBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of remotePortBox as text
%        str2double(get(hObject,'String')) returns contents of remotePortBox as a double


% --- Executes during object creation, after setting all properties.
function remotePortBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to remotePortBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in getTestImBtn.
% function getTestImBtn_Callback(hObject, eventdata, handles)
% % hObject    handle to getTestImBtn (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% remoteIP = get(handles.remoteIPBox,'String');
% remotePort = str2double(get(handles.remotePortBox,'String'));
% localPort = str2double(get(handles.localPortBox,'String'));
% xenicsType = getappdata(handles.AIGui,'xenicsType');
% udpXenics = udp(remoteIP, remotePort, 'LocalPort', localPort);
% fopen(udpXenics);
% 
% if xenicsType == 320
%     m=256;
%     n=320;
% else
%     m=512;
%     n=640;
% end
% imageBytes = m*n*2; % *2 because uint16
% 
% 
% % Send the command to send an image
% cmdVal = uint8(100); % Choose command 100 to be 'send image'
% fwrite(udpXenics, cmdVal)
% 
% % Now wait for a response, with a timeout
% timeout=1; %Seconds
% tic
% err=0;
% while udpXenics.BytesAvailable ~= imageBytes
%     pause(0.01)
%     if toc > timeout
%         err=1;
%         break
%     end
% end
% 
% if err == 1
%     disp('Timeout: failed to receive image')
% else
%     im=fread(udpXenics,[m,n],'uint16');
%     imagesc(im,'Parent',handles.optimAxes)
% end
% 
% fclose(udpXenics)
% delete(udpXenics)



function localPortBox_Callback(hObject, eventdata, handles)
% hObject    handle to localPortBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of localPortBox as text
%        str2double(get(hObject,'String')) returns contents of localPortBox as a double


% --- Executes during object creation, after setting all properties.
function localPortBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to localPortBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in loopCloseBtn.
function loopCloseBtn_Callback(hObject, eventdata, handles)
% hObject    handle to loopCloseBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if getappdata(handles.AIGui,'aiLoopIsClosed')
    % Open the loop
    setappdata(handles.AIGui,'aiLoopIsClosed',false)
    set(handles.loopStatusText,'String','Open');
    set(handles.loopStatusText,'ForegroundColor',[1 0 0]);
else
    % Close the loop
    setappdata(handles.AIGui,'aiLoopIsClosed',true)
    set(handles.loopStatusText,'String','Closed');
    set(handles.loopStatusText,'ForegroundColor',[0 1 0]);
end



function pGainBox_Callback(hObject, eventdata, handles)
% hObject    handle to pGainBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pGainBox as text
%        str2double(get(hObject,'String')) returns contents of pGainBox as a double
pGain=str2double(get(hObject,'String'));
setappdata(handles.AIGui,'pGain',pGain)

% --- Executes during object creation, after setting all properties.
function pGainBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pGainBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function iGainBox_Callback(hObject, eventdata, handles)
% hObject    handle to iGainBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of iGainBox as text
%        str2double(get(hObject,'String')) returns contents of iGainBox as a double


% --- Executes during object creation, after setting all properties.
function iGainBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to iGainBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function dGainBox_Callback(hObject, eventdata, handles)
% hObject    handle to dGainBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dGainBox as text
%        str2double(get(hObject,'String')) returns contents of dGainBox as a double


% --- Executes during object creation, after setting all properties.
function dGainBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dGainBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in doMemsScanBtn.
function doMemsScanBtn_Callback(hObject, eventdata, handles)
% hObject    handle to doMemsScanBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

scanWait=0.1 %Time to wait after MEMS move to send acq command
dataBytes = 8*8; %8 values, doubles
timeout=5; %Timeout to wait for data (s)

segNums = getappdata(handles.AIGui,'segNums');
memsHandle=getappdata(handles.AIGui,'memsHandle');
memsSegsList=getappdata(handles.AIGui,'memsSegsList');
targetPosns=getappdata(handles.AIGui,'targetPosns');
segInds = memsSegsList(segNums);
memsScanRange=getappdata(handles.AIGui,'memsScanRange');
nPosns=length(memsScanRange);

PTTPositionFlat=getappdata(handles.AIGui,'PTTPositionFlat');
SetMirrorPosition(memsHandle, memsSegsList, PTTPositionFlat);
MirrorSendSettings(memsHandle);
pause(scanWait)

remoteIP = get(handles.remoteIPBox,'String');
remotePort = str2double(get(handles.remotePortBox,'String'));
localPort = str2double(get(handles.localPortBox,'String'));
udpXenics = udp(remoteIP, remotePort, 'LocalPort', localPort);
udpXenics.Terminator='';
udpXenics.InputBufferSize = dataBytes;
fopen(udpXenics);

allPhotomValues=zeros(nPosns,nPosns,8);

%for ss = 1:length(segInds)
for ss = 7:7
    seg = segInds(ss);
    for xx = 1:nPosns
        for yy = 1:nPosns
            PTTNewPosition = PTTPositionFlat;
            PTTNewPosition(seg,2) = memsScanRange(xx);
            PTTNewPosition(seg,3) = memsScanRange(yy);
            SetMirrorPosition(memsHandle, memsSegsList, PTTNewPosition);
            MirrorSendSettings(memsHandle);
            set(handles.memsPosnTable,'Data',PTTNewPosition);
            
            pause(scanWait)
            fprintf(udpXenics,'acq');
            
            % Now wait for a response, with a timeout
            tic
            %err=0;
            while udpXenics.BytesAvailable ~= dataBytes
                pause(0.001)
                if toc > timeout
                    %err=1;
                    errordlg('No data received before timeout!','Timeout')
                    break
                end
            end
            data=fread(udpXenics,dataBytes,'double');
            allPhotomValues(xx,yy,:)=data;
        end
        disp(['X position ' num2str(xx) ' of ' num2str(nPosns)])
    end
    disp(['Segment ' num2str(seg)])
end

disp('Finished')

fclose(udpXenics)
delete(udpXenics)

SetMirrorPosition(memsHandle, memsSegsList, PTTPositionFlat);
MirrorSendSettings(memsHandle);




            


% --- Executes on button press in saveDataBtn.
function saveDataBtn_Callback(hObject, eventdata, handles)
% hObject    handle to saveDataBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setappdata(handles.AIGui,'saveDataState',true)
set(hObject,'ForegroundColor',[1 0 0]);


% --- Executes when selected object is changed in ReadoutModeBtns.
function ReadoutModeBtns_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in ReadoutModeBtns 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch get(eventdata.NewValue,'Tag') % Get Tag of selected object.
    case 'NFTBtn'
        ftMode=0;
        disp('FT mode OFF')
        setappdata(handles.AIGui,'ftMode',ftMode)
        error=calllib('atmcd32d','SetFrameTransferMode',ftMode);
    case 'FTBtn'
        ftMode=1;
        disp('FT mode ON')
        setappdata(handles.AIGui,'ftMode',ftMode)
        error=calllib('atmcd32d','SetFrameTransferMode',ftMode);
end

        



function delayTimeBox_Callback(hObject, eventdata, handles)
% hObject    handle to delayTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delayTimeBox as text
%        str2double(get(hObject,'String')) returns contents of delayTimeBox as a double


% --- Executes during object creation, after setting all properties.
function delayTimeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delayTimeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function greyOutCamControls(handles, greyed)
if greyed
    set(handles.expTimeBox,'Enable','off')
    set(handles.emGainBox,'Enable','off')
    set(handles.vsSpeedBox,'Enable','off')
    set(handles.hsSpeedBox,'Enable','off')
    set(handles.vsAmpBox,'Enable','off')
    set(handles.cycTimeBox,'Enable','off')
    set(handles.tempSetBox,'Enable','off')
else
    set(handles.expTimeBox,'Enable','on')
    set(handles.emGainBox,'Enable','on')
    set(handles.vsSpeedBox,'Enable','on')
    set(handles.hsSpeedBox,'Enable','on')
    set(handles.vsAmpBox,'Enable','on')
    set(handles.cycTimeBox,'Enable','on')
    set(handles.tempSetBox,'Enable','on')
end
    
    
    


% --- Executes on button press in saveFrmBtn.
function saveFrmBtn_Callback(hObject, eventdata, handles)
% hObject    handle to saveFrmBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
im=getappdata(handles.AIGui,'currentIm');
filename=[get(handles.saveFrameFilenameBox,'String') '.mat'];
save(filename,'im')


function saveFrameFilenameBox_Callback(hObject, eventdata, handles)
% hObject    handle to saveFrameFilenameBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of saveFrameFilenameBox as text
%        str2double(get(hObject,'String')) returns contents of saveFrameFilenameBox as a double


% --- Executes during object creation, after setting all properties.
function saveFrameFilenameBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to saveFrameFilenameBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
