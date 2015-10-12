
ExpTime = 0.0001
FTmode = 0
numIts = 100;

doCorrection = false;
pGain = 0.5;

bg=417; %Median background count - get from darks

% A position to tilt to, for testing
memsTestPosn = repmat([3, 2, 0],37,1);
%waitTime = 0.001; %wait this long between MEMS movement and image

   
load('winCents.mat');
winSize = 16;  

%For now, set the target positions to the original centres
%targetPosns = winCents;
targetPosns = zeros(8,2);

MirrorSerialNumberString = 'FSC37-02-01-0310';
DriverSerialNumberString = '11140003';
HardwareDisableFlag = false;

doRot = true;
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

%%%%
imCube = zeros(imwidth,imheight,numIts);
%%%%

nSubs = size(winCents,1);
allSubIms = zeros(winSize, winSize, nSubs, numIts);
hSum=zeros(winSize);
vSum=zeros(winSize);
inds=1:winSize;
allCogX = zeros(nSubs, numIts);
allCogY = zeros(nSubs, numIts);

%In proper version these will be 8x arrays
curMemsPosnX=0;
curMemsPosnY=0;
newMemsPosnX=0;
newMemsPosnY=0;

% Main loop
calllib('atmcd32d','StartAcquisition');
tic
for ii = 1:numIts
    
   if ii == 500
       doCorrection = true;
   end
    
   if ~doCorrection
        SetMirrorPosition(memsHandle, memsSegsList, PTTPositionFlat); %Flatten MEMS
        MirrorSendSettings(memsHandle);
   end
    
%     if ii == 501
%         SetMirrorPosition(memsHandle, memsSegsList, memsTestPosn);
%         MirrorSendSettings(memsHandle);
%     end

    
    error=calllib('atmcd32d','WaitForAcquisition');
    getimerr=calllib('atmcd32d','GetMostRecentImage',imPtr,npixels);
    im=imPtr.value - bg;

    %%%%
    imCube(:,:,ii)=im;
    %%%%
    
    % Loops through subims, but can do this faster with some array op
    for kk = 1:nSubs
        allSubIms(:,:,kk,ii) = im(winCents(kk,1)-winSize/2:winCents(kk,1)+winSize/2-1, ...
            winCents(kk,2)-winSize/2:winCents(kk,2)+winSize/2-1);
        
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
    
    %Do correction
    if doCorrection
        measX = allCogX(1,ii);
        measY = allCogY(1,ii);
        errorX = measX - targetPosns(1,1);
        errorY = measY - targetPosns(1,2);
        newMemsPosnX = curMemsPosnX - errorX*pGain;
        newMemsPosnY = curMemsPosnY - errorY*pGain;
        memsPosnTable = repmat([0, newMemsPosnX, newMemsPosnY],37,1);
        SetMirrorPosition(memsHandle, memsSegsList, memsPosnTable);
        MirrorSendSettings(memsHandle);
        curMemsPosnX = newMemsPosnX;
        curMemsPosnY = newMemsPosnY;
    end
    
    %pause(0.08)
    %pause(1e-6)
    
    if mod(ii,100) == 0
        disp(ii)
    end
     pause(1e-16)
%    pause(0.1)
end
T=toc;
disp(strcat('Average time per frame: ',num2str(T/numIts)))


calllib('atmcd32d','AbortAcquisition');
        

        
        
% Close shutter
error=calllib('atmcd32d','SetShutter',0,2,10,10);
  
error=calllib('atmcd32d','ShutDown');
pause(1)
unloadlibrary atmcd32d;
    
    
% Release MEMS
MirrorRelease(memsHandle)
    
    




    
