% UDP test

%For computer 1:
%%% Define computer-specific variables
ipA = '10.0.1.6';
ipA = '10.80.85.10';   portA = 9090;   % Modify these values to be those of your first computer.
ipB = '10.80.85.12';  portB = 9091;  % Modify these values to be those of your second computer.
%%% Create UDP Object
udpA = udp(ipB,portB,'LocalPort',portA);
%%% Connect to UDP Object
fopen(udpA)


%Later:
fprintf(udpA,'This is test message number one.')
fprintf(udpA,'This is test message number two.')

%Also,
fwrite(udpA,3.14159,'double')





%For computer 2:
%%% Define computer-specific variables
ipA = '10.80.85.10';   portA = 9090;   % Modify these values to be those of your first computer.
ipB = '10.80.85.12';  portB = 9091;  % Modify these values to be those of your second computer.
%%% Create UDP Object
udpB = udp(ipA,portA,'LocalPort',portB);
%%% Connect to UDP Object
fopen(udpB)


%Later:
udpB.BytesAvailable
fscanf(udpB)

%Also,
fread(udpB,8,'double')


%%% Clean Up Machine A
fclose(udpA)
delete(udpA)
clear ipA portA ipB portB udpA

%%% Clean Up Machine B
fclose(udpB)
delete(udpB)
clear ipA portA ipB portB udpB


