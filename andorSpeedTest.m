%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define parameters
%

ExpTime = 0%0.005%0.05;      % Exposure time
                    % Note - in Ext Trigger mode, exposure controlled by
                    % time between triggers, but exposure window is moved
                    % by ExpTime.

                    
numVidFrames = 1000; % Number of frames to show in video mode before exiting

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

% Set read mode to Image
error=calllib('atmcd32d','SetReadMode',4);

% Set acquisition mode to Run til abort
error=calllib('atmcd32d','SetAcquisitionMode',5);
error=calllib('atmcd32d','SetExposureTime',ExpTime);

%error=calllib('atmcd32d','SetNumberAccumulations',1);
%error=calllib('atmcd32d','SetNumberKinetics',NumFrames);
% Following line redundant in FT mode (depends on exptime, cannot set this in software)
%error=calllib('atmcd32d','SetKineticCycleTime',0); %Use shortest possible cycle time
% Use GetAcquisitionTimings to find actual timings...
%error=calllib('atmcd32d','SetKineticCycleTime',0.1);



% Set frame transfer mode to off
error=calllib('atmcd32d','SetFrameTransferMode',0);

% Set trigger mode to internal
error=calllib('atmcd32d','SetTriggerMode',0);

% Set shift speeds (to do)

% Set shutter to open
error=calllib('atmcd32d','SetShutter',1,1,50,50);

% Setup image size
imw=libpointer('int32Ptr',0);
imh=libpointer('int32Ptr',0);
error=calllib('atmcd32d','GetDetector',imw,imh);
imwidth=imw.value;
imheight=imw.value;
error=calllib('atmcd32d','SetImage',1,1,1,imwidth,1,imheight);


%Print the actual cycle time - for now just to console.
actualExp=libpointer('singlePtr',0);
actualAcc=libpointer('singlePtr',0);
actualCyc=libpointer('singlePtr',0);
error=calllib('atmcd32d','GetAcquisitionTimings',actualExp,actualAcc,actualCyc);
disp(strcat('Actual cycle time: ',num2str(actualCyc.value)))
disp(strcat('Actual exposure time: ',num2str(actualExp.value)))

npixels=imwidth*imheight;
imPtr = libpointer('int32Ptr',zeros(imwidth,imheight));

imCube = zeros(imwidth,imheight,numVidFrames);
%%% maxData=zeros(3,numVidFrames);

calllib('atmcd32d','StartAcquisition');
tic
for ii = 1:numVidFrames
    error=calllib('atmcd32d','WaitForAcquisition');
    
    getimerr=calllib('atmcd32d','GetMostRecentImage',imPtr,npixels);
    im=imPtr.value;
    %imCube(:,:,ii) = imPtr.value;
    imCube(:,:,ii) = im;
    
%%%    maxData(1:2,ii) = max(im(:));
%%%    maxData(3,ii) = mean(im(:));

%    pause(0.01)
end
T=toc;
disp(strcat('Average time per frame: ',num2str(T/numVidFrames)))
calllib('atmcd32d','AbortAcquisition');
        

        
        
% Close shutter
error=calllib('atmcd32d','SetShutter',0,2,10,10);
    
error=calllib('atmcd32d','ShutDown');
unloadlibrary atmcd32d;