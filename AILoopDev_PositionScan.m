
ExpTime = 0.001 
FTmode = 1
%numIts = 1000;

showIms = false;

bg=417; %Median background count - get from darks
doAutoBG = false;
bgRegion = [ 55,75 ; 55, 75]; %row;col


% This version scans through tip or tilt
angles=-3:0.01:3;
%angles=-1:0.01:1;
%angles=-2.5:0.01:2.5;
%angles=-5:0.01:5;
%angles=-5:0.1:5
%angles=-5:1:5

tipTiltIndex = 3   ; %2 or 3
waitTime = 0.04; %wait this long between MEMS movement and image
numIts = length(angles);
cooling = true;


% A position to tilt to, for testing
memsTestPosn = repmat([3, 2, 0],37,1);


%winCents = [100, 102; ...
%               103, 34] ;      
load('winCents.mat');
winSize = 16;  

%For now, set the target positions to the original centres
targetPosns = winCents;

MirrorSerialNumberString = 'FSC37-02-01-0310';
DriverSerialNumberString = '11140003';
HardwareDisableFlag = false;


doRot = false;
setTheta = 60 /180*pi;
mX = -0.38553;
bX = -3.2958;
mY = -0.43751;
bY = 12.0208;
Rmat = [ cos(setTheta), -sin(setTheta) ; sin(setTheta), cos(setTheta) ];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
header='C:\Program Files\Andor iXon\Drivers\ATMCD32D.H';
[notfound, warnings] = loadlibrary('C:\Program Files\Andor iXon\Drivers\atmcd32d.dll', header);

% Connect to MEMS and turn it on
disp('Initaliasing MEMS...')
memsHandle = MirrorConnect(MirrorSerialNumberString, ...
    DriverSerialNumberString, HardwareDisableFlag);
numMemsSegs=37;
PTTPositionFlat = zeros(numMemsSegs,3);
memsSegsList=1:numMemsSegs;
SetMirrorPosition(memsHandle, memsSegsList, PTTPositionFlat); %Flatten MEMS
MirrorSendSettings(memsHandle);

% Initalise camera
disp('Initaliasing Camera...')
error=calllib('atmcd32d','Initialize','C:\Program Files\Andor iXon\Drivers');

if error == 20002
    disp('Initialisation Succesful')
else
    disp('Initialisation Error')
end

if cooling
    targetTemp=-40;
    error=calllib('atmcd32d','SetFanMode',0); %0 is full on.
    error=calllib('atmcd32d','SetTemperature',targetTemp-5);
    error=calllib('atmcd32d','CoolerON');
    
    curTempPtr=libpointer('int32Ptr',0);
    tempError=calllib('atmcd32d','GetTemperature',curTempPtr);
    curTemp=curTempPtr.value;
    while curTemp > (targetTemp + 5)
        disp('Waiting for camera to cool...')
        disp(['Current temp: ' num2str(curTemp)])
        pause(5)
        tempError=calllib('atmcd32d','GetTemperature',curTempPtr);
        curTemp=curTempPtr.value;
    end
    disp('Temperature OK')
end




% Set shutter to open
error=calllib('atmcd32d','SetShutter',1,1,50,50);

% Set read mode to Image
error=calllib('atmcd32d','SetReadMode',4);

% Set acquisition mode to Run til abort
error=calllib('atmcd32d','SetAcquisitionMode',5);
error=calllib('atmcd32d','SetExposureTime',ExpTime);

% Set frame transfer mode to off
error=calllib('atmcd32d','SetFrameTransferMode',0);

% Set trigger mode to internal
error=calllib('atmcd32d','SetTriggerMode',0);

% Setup image size
imw=libpointer('int32Ptr',0);
imh=libpointer('int32Ptr',0);
error=calllib('atmcd32d','GetDetector',imw,imh);
imwidth=imw.value;
imheight=imw.value;
npixels=imwidth*imheight;
imPtr = libpointer('int32Ptr',zeros(imwidth,imheight));
error=calllib('atmcd32d','SetImage',1,1,1,imwidth,1,imheight);

imCube = zeros(imwidth,imheight,numIts);
nSubs = size(winCents,1);
allSubIms = zeros(winSize, winSize, nSubs, numIts);
hSum=zeros(winSize);
vSum=zeros(winSize);
inds=1:winSize;
allCogX = zeros(nSubs, numIts);
allCogY = zeros(nSubs, numIts);

% Main loop
calllib('atmcd32d','StartAcquisition');
tic
for ii = 1:numIts
    
    angle=angles(ii);
    memsPosn=zeros(numMemsSegs,3);
    memsPosn(:,tipTiltIndex) = angle;
    SetMirrorPosition(memsHandle, memsSegsList, memsPosn);
    MirrorSendSettings(memsHandle);   
    pause(waitTime)
    
    error=calllib('atmcd32d','WaitForAcquisition');
    getimerr=calllib('atmcd32d','GetMostRecentImage',imPtr,npixels);
    im = imPtr.value;
    imCube(:,:,ii) = im;
    
    bgImage = im(bgRegion(1,1):bgRegion(1,2),bgRegion(2,1):bgRegion(2,2));
    bg = mean(bgImage(:));
    
    im=im - bg;
    
    % Loops through subims, but can do this faster with some array op
    for kk = 1:nSubs
        allSubIms(:,:,kk,ii) = im(winCents(kk,1)-winSize/2:winCents(kk,1)+winSize/2-1, ...
            winCents(kk,2)-winSize/2:winCents(kk,2)+winSize/2-1);
        
        if showIms
                subplot(4,2,kk)
                imagesc(allSubIms(:,:,kk,ii))
                axis equal
        end
        
        % Measure COGs
        hSum=sum(allSubIms(:,:,kk,ii), 2);
        vSum=sum(allSubIms(:,:,kk,ii), 1);
        %allCogX(kk,ii) = sum(vSum.*inds)/sum(vSum);
        %allCogY(kk,ii) = sum(hSum'.*inds)/sum(hSum);
        cur = [sum(vSum.*inds)/sum(vSum) ; sum(hSum'.*inds)/sum(hSum)];       
        if doRot
            cur = Rmat*cur;
            cur(1) = (cur(1) - bX)/mX;
            cur(2) = (cur(2) - bY)/mY;
        end
        allCogX(kk,ii) = cur(1);
        allCogY(kk,ii) = cur(2);  
  
    
    end
    if showIms
        %pause(0.5)
        drawnow
    end
end
T=toc;
disp(strcat('Average time per frame: ',num2str(T/numIts)))


calllib('atmcd32d','AbortAcquisition');
       

save(['sampledata_scan_autosave_' datestr(now,30) '.mat'],'allCogX','allCogY','allSubIms','angles','imCube')

disp('Finished - press any key to shutdown')
pause
        
        
% Close shutter
error=calllib('atmcd32d','SetShutter',0,2,10,10);
  
pause(3)

error=calllib('atmcd32d','ShutDown');
%unloadlibrary atmcd32d;
    
    
% Release MEMS
MirrorRelease(memsHandle)
    

    
