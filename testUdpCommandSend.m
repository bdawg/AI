% This would be in the AI gui code.

%xenicsType = 640
nIts = 100; 

remoteIP = '10.88.18.2' 
remotePort = 9091 
localPort = 9090 

% if xenicsType == 320
%     m=256;
%     n=320;
% else
%     m=512;
%     n=640;
% end

udpXenics = udp(remoteIP, remotePort, 'LocalPort', localPort);
udpXenics.Terminator='';
% udpAI.OutputBufferSize = m*n*2;
fopen(udpXenics);

for k = 1:nIts
    pause
    disp('Sending command');
    fprintf(udpXenics,'acq');
    pause(0.01)
    
end




% % A loop to simulate as timer in the gui
% disp('Beginning loop')
% 
% for k = 1:nIts  
% 
%     % Make a fake image
%     im = uint16(imnoise(zeros(m,n),'gaussian',0.5,0.01)*1e4);
%     %im=im1;
%     
%     %Check for a command byte
%     if udpXenics.BytesAvailable > 0
%         cmdIn = fread(udpXenics);
%         
%         % TODO - if cmd more than one byte, report an error
%         if length(cmdIn) == 1
%             
%             % Here you could respond to several different commands.
%             switch cmdIn
%                 case 100 %Send an image
%                     disp('Sending an image')
%                     fwrite(udpXenics, im(:), 'uint16')
%                     break
%                 case 1
%                     disp('received cmd number 1')
%                     
%                 otherwise
%                     disp('Received unknown command')
%                     
%             end
%             
%         end
%                   
%     end
%         
%     k
%     pause(0.1)
% end
        

fclose(udpXenics)
delete(udpXenics)

