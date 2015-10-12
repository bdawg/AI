%from function getTestImBtn_Callback(hObject, eventdata, handles)

xenicsType = 640

remoteIP = '10.0.1.6' %get(handles.remoteIPBox,'String');
remoteIP = '10.0.1.15'
remotePort = 9090 %str2double(get(handles.remotePortBox,'String'));
localPort = 9091 %str2double(get(handles.localPortBox,'String'));
%xenicsType = getappdata(handles.AIGui,'xenicsType');
udpXenics = udp(remoteIP, remotePort, 'LocalPort', localPort);


if xenicsType == 320
    m=256;
    n=320;
else
    m=512;
    n=640;
end
imageBytes = m*n*2; % *2 because uint16
im = uint16(zeros(m,n));

udpXenics.InputBufferSize = imageBytes;
udpXenics.DatagramTerminateMode = 'off';
fopen(udpXenics);


% Send the command to send an image
cmdVal = uint8(100); % Choose command 100 to be 'send image'
fwrite(udpXenics, cmdVal)

% Now wait for a response, with a timeout
timeout=5; %Seconds
tic
err=0;
while udpXenics.BytesAvailable ~= imageBytes
    pause(0.01)
    if toc > timeout
        err=1;
        break
    end
    udpXenics.BytesAvailable
end
%udpXenics.BytesAvailable


if err == 1
    disp('Timeout: failed to receive image')
else
    toc
    tic
    im=fread(udpXenics,[m,n],'uint16');
    toc
    %in=fread(udpXenics,imageBytes/2,'uint16');
    %imagesc(im,'Parent',handles.optimAxes)
    imagesc(im)
end

fclose(udpXenics)
delete(udpXenics)