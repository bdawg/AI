function Twitch(MirrorSerialNumberString, DriverSerialNumberString, SegmentList, Magnitude, DirectionString, Period, NumCycles, HardwareDisableFlag);
%==========================================================================
%	Twitch.m
%
%	Copyright© 2015 Iris AO, Inc.
%
%	Author: Michael A. Helmbrecht
%
%	Origination Date: 4/23/2015
%
%	Description:	
%       Twitches a set of segments a defined angle plus and minus from
%       flat. The array is first flattened. Twitching begins after the
%       users presses the "Enter" key.
%
%	Function Format: 
%		Twitch(MirrorSerialNumberString, DriverSerialNumberString, SegmentList, 
%                   Magnitude, DirectionString, Period, NumCycles, HardwareDisableFlag)
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
%		Magnitude: The angle to twitch the segments in mrad.
%
%       DirectionString: The direction to twitch the segments. Valid entries
%           are 'Horizontal' (grad X direction) or 'Vertical' (grad Y
%           direction). The parsing only looks for 'h' or 'v' in the
%           string. So, 'h' and 'H' are valid entries for a horizontal
%           (grad X) twitch. Likewise, 'v' and 'V' are valid entries for a
%           vertical (grad Y) twitch.
%
%       Period: The nominal twitch period in seconds.
%
%       NumCycles: The number of times to twitch the segment
%
%       HardwareDisableFlag: A boolean value declaring whether to enable or
%           disable the hardware. Disabling the hardware allows the user to
%           test functionality without hardware connected.
%	
%   Usage: Twitch('PWA163-02-09-0210', '13100037', [1 110 92], 1.0, 'H', 0.5, 10, true);
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

% Check for direction
if ~isempty(strfind(lower(DirectionString), 'h'))
    Direction = 2;
else if ~isempty(strfind(lower(DirectionString), 'v'))
        Direction = 3;
    else
        disp('ERROR: Invalid direction format. Should be either "H[orizontal]" or "V[ertical]"')
        LockedSegments = [];
        UnreachableSegments = [];
        return
    end
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
% Create the position arrays for twitching
%-------------------------------------------------------------------
PTTPositionFlat = zeros(NumSegments,3);
PTTPositionPositive = zeros(NumSegments,3);
PTTPositionPositive(SegmentList, Direction) = Magnitude;
PTTPositionNegative = -1*PTTPositionPositive;


%===================================================================
%==============        Position Phase - Start         ==============
%===================================================================
%-------------------------------------------------------------------
% Flatten the mirror
%-------------------------------------------------------------------
% Load the positions into the internal buffer
SetMirrorPosition(Handle, 1:NumSegments, PTTPositionFlat);

% Send the positions to the mirror
MirrorSendSettings(Handle);

% Notify the user that the positions were set and wait to twitch
input('Mirror flattened, press "Enter" to start twitching');


%-------------------------------------------------------------------
% Twitch loop 
%-------------------------------------------------------------------
for TwitchNum = 1:NumCycles
    % Positive direction
    SetMirrorPosition(Handle, 1:NumSegments, PTTPositionPositive);
    MirrorSendSettings(Handle);
    [ReachablePositionsPositive LockedFlag ReachableFlagPositive] = ...
        GetMirrorPosition(Handle, 1:NumSegments);
    pause(Period/2);
    
    % Negative direction
    SetMirrorPosition(Handle, 1:NumSegments, PTTPositionNegative);
    MirrorSendSettings(Handle);
    [ReachablePositionsNegative LockedFlag ReachableFlagNegative] = ...
        GetMirrorPosition(Handle, 1:NumSegments);
    pause(Period/2);
end
%===================================================================
%==============         Position Phase - End          ==============
%===================================================================

% Notify the user that twitching stopped
input('Twitching complete, press "Enter" to shutdown');


%===================================================================
%==============          Release Phase - Start        ==============
%===================================================================
MirrorRelease(Handle)
%===================================================================
%==============           Release Phase - End         ==============
%===================================================================


%-------------------------------------------------------------------
% Check for locked and uncreachable twitch segments 
%-------------------------------------------------------------------
% Locked segments
LockedSegments = find(LockedFlag);

% Remove unused segments from the list
LockedSegments = setdiff(LockedSegments, PTT489UnusedSegments);

% Determine if any of the twitch segments were locked
LockedTwitchSegments = intersect(SegmentList, LockedSegments);

if ~isempty(LockedTwitchSegments)
    disp('WARNING: The following twitch segments were locked')
    disp(LockedTwitchSegments)
end

% Find unreachable segments - exclude locked segments and unused segments
UnreachableSegmentsPositive = ...
    find((LockedFlag + ReachableFlagPositive) == 0);
UnreachableSegmentsNegative = ...
    find((LockedFlag + ReachableFlagNegative) == 0);
UnreachableTwitchSegmentsPositive = ...
    intersect(SegmentList, UnreachableSegmentsPositive);
UnreachableTwitchSegmentsNegative = ...
    intersect(SegmentList, UnreachableSegmentsNegative);

% Report unreachable segments - positive direction
if ~isempty(UnreachableTwitchSegmentsPositive)
    disp('WARNING: These twitch segments were unreachable for the positive tilt')
    disp(UnreachableTwitchSegmentsPositive)
end

% Report unreachable segments - negative direction
if ~isempty(UnreachableTwitchSegmentsNegative)
    disp('WARNING: These twitch segments were unreachable for the negative tilt')
    disp(UnreachableTwitchSegmentsNegative)
end