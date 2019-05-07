# Minimum-Time-To-Climb

This was a project I created for my senior design course in Aerospace Engineering. My team created a vertical takeoff and landing (VTOL) unmanned aerial vehicle (UAV) with a flight plan to take-off vertically 15m and then reach a cruise altitude of 63m. For the sake of this project, I wanted to create the optimal flight path for the UAV to rise from it's transiton stage at 15m to the cruise altitude of 63m. To do so, I used the TomLab optimization package with Matlab.

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Before running this code, TomLab must be installed. To get a 3 month    %
% trial of TomLab, visit https://tomopt.com/tomlab/ After starting Matlab % 
% for the first time after installing TomLab, navigate to the .\tomlab    % 
% folder where the license is installed, and run the 'startup.m' file.    %
% From here, this script can then be run.                                 %
%                                                                         %
% This script will minimize the time for a quadplane to travel from hover %
% at 10m to cruise at 61m. The final cruise state will have a flight      %
% angle of attack of 0 degrees and a cruise velocity of 24 m/s.           %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
