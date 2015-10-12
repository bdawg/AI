
ExpTime = 0.005 
FTmode = 0
%numIts = 1000;

showIms = false;
cooling = true;

%bg=417; %Median background count - get from darks





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

if cooling
    targetTemp=-65;
    error=calllib('atmcd32d','SetFanMode',0); %0 is full on.
    error=calllib('atmcd32d','SetTemperature',targetTemp);
    error=calllib('atmcd32d','CoolerON');
    
    curTempPtr=libpointer('int32Ptr',0);
    tempError=calllib('atmcd32d','GetTemperature',curTempPtr);
    curTemp=curTempPtr.value;
    while curTemp > targetTemp
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



% Main loop
calllib('atmcd32d','StartAcquisition');
error=calllib('atmcd32d','WaitForAcquisition');
getimerr=calllib('atmcd32d','GetMostRecentImage',imPtr,npixels);
im=imPtr.value;
calllib('atmcd32d','AbortAcquisition');
        
save('saveWFSImage_autosave','im')
disp('Image acquired.')
pause
        
        
% Close shutter
error=calllib('atmcd32d','SetShutter',0,2,10,10);
  
pause(3)

error=calllib('atmcd32d','ShutDown');
%unloadlibrary atmcd32d;
    

    
    
    
