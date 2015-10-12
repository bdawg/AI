function [LockedSegments, UnreachableSegments] = ZonalSet(MirrorSerialNumberString, DriverSerialNumberString, SegmentList, PTTPositionArray, HardwareDisableFlag);
%==========================================================================
%	ZonalSet.m
%
%	Copyright© 2015 Iris AO, Inc.
%
%	Author: Michael A. Helmbrecht
%
%	Origination Date: 4/23/2015
%
%	Description:	
%       Sets a mirror position for the specified segments.
%
%	Function format: 
%		ZonalSet(MirrorSerialNumberString, DriverSerialNumberString,
%                       		SegmentList, PTTPositionArray, HardwareDisableFlag);
%
%	Function Arguments:
%		MirrorSerialNumberString: The name of the .mcf file without the .mcf
%           extension. The file name is made of the mirror serial number 
%           and potentialy some additional descriptive text.
%
%		DriverSerialNumberString: The driver .dcf file name without the .dcf
%           extension. The file name is made of the driver serial number 
%           and potentialy some additional descriptive text.
%
%		SegmentList: The segments to actuate. An [Nx1]array of segment IDs.
%
%		PTTPositionArray: The piston/tip/tilt positions to send the mirror
%           segment(s) to. An [Nx3] in [µm, mrad, mrad].
%
%       HardwareDisableFlag: A boolean value declaring whether to enable or
%           disable the hardware. Disabling the hardware allows the user to
%           test functionality without hardware connected.
%
%   Funtion Outputs:
%       LockedSegments: A vector of the locked segments.
%
%       UnreachableSegments: A vector of the unreachable segments.
%	
%   Usage: [LockedSegments, UnreachableSegments] = ZonalSet('PWA163-02-09-0210', '13100037', 1:169, zeros(169,3), true);
%
%
%   Version: 0.0.0.0
%
%	NOTES:
%   o  For PTT489 and larger arrays, unused segments are reported as locked
%   and unreachable from the GetMirrorPosition call. The unused segments
%   must be removed to get accurate counts.
%
%
%   REVISIONS:
%
%   BUGS:
%   o  As written, the function only reports the number of unreachable and
%   unlocked segments correctly for the PTT111 and PTT489 DMs.
% 
%==========================================================================


%-------------------------------------------------------------------
% Variable Declarations & Initialization
%-------------------------------------------------------------------
PTT489UnusedSegments = [128 135 142 149 156 163];


%-------------------------------------------------------------------
% Condition the function parameters
%-------------------------------------------------------------------
% Verify the extensions have not been included for the MCF
MCFIndex = strfind(MirrorSerialNumberString,'.mcf');
if ~isempty(MCFIndex)
    disp('WARNING: Removing the .mcf extension')
    MirrorSerialNumberString = MirrorSerialNumberString(1:MCFIndex-1);
end

% Verify the extensions have not been included for the DCF
DCFIndex = strfind(DriverSerialNumberString,'.mcf');
if ~isempty(DCFIndex)
    disp('WARNING: Removing the .dcf extension')
    DriverSerialNumberString = DriverSerialNumberString(1:DCFIndex-1);
end



%===================================================================
%==============        Connect Phase - Start          ==============
%===================================================================

Handle = MirrorConnect(MirrorSerialNumberString, ...
    DriverSerialNumberString, HardwareDisableFlag);

%===================================================================
%==============         Connect Phase - End           ==============
%===================================================================


%-------------------------------------------------------------------
% Check to see if the DM is a PTT111 or a PTT489
%-------------------------------------------------------------------
% If (in Matlab) a segment ID that does not exist is checked, the MEX file
% will send an exception
try
    % Get the positions for segment 38. If the DM is a PTT111, this will
    % throw an exception an denter the catch block. If it is a PTT489
    % (or a derivative of the PTT489), it will execute
    GetMirrorPosition(Handle, 38);
    NumSegments = 169;
catch
    NumSegments = 37;
end


%-------------------------------------------------------------------
% Condition the segment list - make sure they all exist
%-------------------------------------------------------------------
if max(SegmentList) > NumSegments || min(SegmentList) < 1
    disp('WARNING: Segment list includes non-existent segment IDs')
    disp('WARNING: Reducing segment list valid segments only')
    % eliminate segments outside of the existent segments
    SegmentList = intersect(SegmentList, [1:NumSegments]);
end



%-------------------------------------------------------------------
% Create the position array
%-------------------------------------------------------------------
PTTPositions = zeros(NumSegments,3);
PTTPositions(SegmentList,:) = PTTPositionArray;


%===================================================================
%==============        Position Phase - Start         ==============
%===================================================================
% Load the positions into the internal buffer
SetMirrorPosition(Handle, 1:NumSegments, PTTPositions);

% Send the positions to the mirror
MirrorSendSettings(Handle);

% Get the actual (reachable) segment positions
[ReachablePositions LockedFlag ReachableFlag] = GetMirrorPosition(Handle, SegmentList);  

%===================================================================
%==============         Position Phase - End          ==============
%===================================================================

% Notify the user that the positions were set and wait to twitch
input('Mirror positioned, press "Enter" to shut down');


%===================================================================
%==============          Release Phase - Start        ==============
%===================================================================
MirrorRelease(Handle)
%===================================================================
%==============           Release Phase - End         ==============
%===================================================================



%-------------------------------------------------------------------
% Report the number of locked and unreachable segments
%-------------------------------------------------------------------
% Locked segments
LockedSegments = find(LockedFlag);

% Remove unused segments from the list
LockedSegments = setdiff(LockedSegments, PTT489UnusedSegments);

% Report number of locked segments
NumLocked = length(LockedSegments);
PrintString = sprintf('Number of locked segments: %i', NumLocked);
disp(PrintString);

% Report number of unreachable segments
UnreachableSegments = find((LockedFlag + ReachableFlag) == 0); % Exclude the unused segments
NumUnreachable = length(UnreachableSegments);
PrintString = sprintf('Number of segments outside of the reachable space: %i', NumUnreachable);
disp(PrintString);