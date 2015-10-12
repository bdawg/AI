
ExpTime = 0.0001
FTmode = 0
numIts = 1000;

%winCents = [100, 102; ...
%               103, 34] ;

winSize = 16;
           
load('winCents.mat');

%For now, set the target positions to the original centres
targetPosns = winCents;

% Example syntax for MEMS:
%Twitch('FSC37-02-01-0310', '11140003', [1:37], 3.0, 'H', 0.5, 10, false)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

header='C:\Program Files\Andor iXon\Drivers\ATMCD32D.H';
[notfound, warnings] = loadlibrary('C:\Program Files\Andor iXon\Drivers\atmcd32d.dll', header);

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
    error=calllib('atmcd32d','WaitForAcquisition');
    getimerr=calllib('atmcd32d','GetMostRecentImage',imPtr,npixels);
    im=imPtr.value;

    % Loops through subims, but can do this faster with some array op
    for kk = 1:nSubs
        allSubIms(:,:,kk,ii) = im(winCents(kk,1)-winSize/2:winCents(kk,1)+winSize/2-1, ...
            winCents(kk,2)-winSize/2:winCents(kk,2)+winSize/2-1);
        
        % Measure COGs
        hSum=sum(allSubIms(:,:,kk,ii), 2);
        vSum=sum(allSubIms(:,:,kk,ii), 1);
        allCogX(kk,ii) = sum(vSum.*inds)/sum(vSum);
        allCogY(kk,ii) = sum(hSum'.*inds)/sum(hSum);    
    
    end
end
T=toc;
disp(strcat('Average time per frame: ',num2str(T/numIts)))


calllib('atmcd32d','AbortAcquisition');
        

        
        
% Close shutter
error=calllib('atmcd32d','SetShutter',0,2,10,10);
  
error=calllib('atmcd32d','ShutDown');
unloadlibrary atmcd32d;
    
    
    
    
    
