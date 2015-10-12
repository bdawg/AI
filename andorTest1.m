%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define parameters
%
NumFrames = 100;     % Number of scans in kinetic series
ExpTime = 0.05;      % Exposure time
                    % Note - in Ext Trigger mode, exposure controlled by
                    % time between triggers, but exposure window is moved
                    % by ExpTime.

Mode =1;           % Mode of this script
                    % 1 - Acquire images into memory
                    % 2 - Spool to fits file
                    % 3 - Acquire single sets and show (testing)
                    % 4 - Show 'video' via Run Till Abort
                    
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

% Set acquisition mode to Kinetic Series and related settings
error=calllib('atmcd32d','SetAcquisitionMode',3);
error=calllib('atmcd32d','SetExposureTime',ExpTime);
error=calllib('atmcd32d','SetNumberAccumulations',1);
error=calllib('atmcd32d','SetNumberKinetics',NumFrames);
% Following line redundant in FT mode (depends on exptime, cannot set this in software)
%error=calllib('atmcd32d','SetKineticCycleTime',0); %Use shortest possible cycle time
% Use GetAcquisitionTimings to find actual timings...
error=calllib('atmcd32d','SetKineticCycleTime',0.1);

% Set frame transfer mode to on
error=calllib('atmcd32d','SetFrameTransferMode',1);

% Set trigger mode to internal
error=calllib('atmcd32d','SetTriggerMode',0);

% Set shift speeds (to do)

% Set shutter to open
error=calllib('atmcd32d','SetShutter',0,1,50,50);

% Setup image size
imw=libpointer('int32Ptr',0);
imh=libpointer('int32Ptr',0);
error=calllib('atmcd32d','GetDetector',imw,imh);
imwidth=imw.value;
imheight=imw.value;
error=calllib('atmcd32d','SetImage',1,1,1,imwidth,1,imheight);


switch Mode
    
    case 1
     %for test = 1:10
        npixels=imwidth*imheight;
        imCube = zeros(imwidth,imheight,NumFrames);
        imPtr = libpointer('int32Ptr',zeros(imwidth,imheight));
        count=1;    
    
        error=calllib('atmcd32d','StartAcquisition');
     
        % Loop until all image have been retrieved and acquisition has finished
        status = 20072; %Initialise with state 'DRV_ACQUIRING'
        getimerr = 20002; %Initialise with state 'DRV_SUCCESS'
        while ((status == 20072) || (getimerr == 20002))
            statptr=libpointer('int32Ptr',0);
            error=calllib('atmcd32d','GetStatus',statptr);
            status=statptr.value;       
            getimerr=calllib('atmcd32d','GetOldestImage',imPtr,npixels);
            if getimerr == 20002
                imCube(:,:,count) = imPtr.value;
                count=count+1
            end
            
%             newimindstartptr=libpointer('int32Ptr',0);
%             newimindendptr=libpointer('int32Ptr',0);
%             calllib('atmcd32d','GetNumberNewImages',newimindstartptr,newimindendptr);
%             disp(newimindstartptr.value)
%             disp(newimindendptr.value)

%               nImAcq=libpointer('int32Ptr',0);
%               error=calllib('atmcd32d','GetTotalNumberImagesAcquired',nImAcq);
%               disp(nImAcq.value)
        end
        
        %end
        %error=calllib('atmcd32d','WaitForAcquisition');
        %%%%%%%% Can't get 'GetAcquiredData to work (always complains wrong
        %%%%%%%% array size).
        % Get number of images acquired
        %nImAcq=libpointer('int32Ptr',0);
        %error=calllib('atmcd32d','GetTotalNumberImagesAcquired',nImAcq);
        
        
    case 2
        error=calllib('atmcd32d','SetSpool',1,5,'fitsspool',10);
        error=calllib('atmcd32d','StartAcquisition');
        % WaitForAcquisition doesn't seem to wait long enough? Use a loop:
        status = 20072; %Initialise with state 'DRV_ACQUIRING'
        count=1;
        while status == 20072
            statptr=libpointer('int32Ptr',0);
            error=calllib('atmcd32d','GetStatus',statptr);
            status=statptr.value
            pause(0.01)
            %count=count+1
        end
        disp('done')
        error=calllib('atmcd32d','SetSpool',0,5,'fitsspool',10);

   
        
    case 3
        % This is a slow way of doing 'video'
        error=calllib('atmcd32d','SetNumberKinetics',1);
        npixels=imwidth*imheight;
        imPtr = libpointer('int32Ptr',zeros(imwidth,imheight));
        for ii = 1:numVidFrames
            calllib('atmcd32d','StartAcquisition');
            %calllib('atmcd32d','WaitForAcquisition');
            calllib('atmcd32d','WaitForAcquisitionTimeOut',1000);
            
            statptr=libpointer('int32Ptr',0);
            error=calllib('atmcd32d','GetStatus',statptr);
            disp(statptr.value)
            
            %getimerr=calllib('atmcd32d','GetOldestImage',imPtr,npixels);
            % Get all images before moving on
            getimerr=2002;
            while getimerr == 2002
                getimerr=calllib('atmcd32d','GetOldestImage',imPtr,npixels);
            end
            
            imagesc(imPtr.value)
            axis square
            pause(0.01)
        end
            disp('Got out of loop')
    
            
            
    case 4
        % Better video method
        npixels=imwidth*imheight;
        imPtr = libpointer('int32Ptr',zeros(imwidth,imheight));
        calllib('atmcd32d','SetAcquisitionMode',5); %run till abort
        calllib('atmcd32d','StartAcquisition');
        figure;
        h=axes;
        for ii = 1:numVidFrames
            getimerr=calllib('atmcd32d','GetMostRecentImage',imPtr,npixels);
            imagesc(imPtr.value,'Parent',h)
            axis square
            
%             statptr=libpointer('int32Ptr',0);
%             error=calllib('atmcd32d','GetStatus',statptr);
%             disp(statptr.value)
%             ii
            
            pause(0.01)
        end
        calllib('atmcd32d','AbortAcquisition');
        
end
        
        
% Close shutter
error=calllib('atmcd32d','SetShutter',0,2,10,10);
    
error=calllib('atmcd32d','ShutDown');
unloadlibrary atmcd32d;