clear; clc; close all
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

% Initial and final values for state variables altitude [m], 
% velocity [m/s], and flight angle of attack (gamma) [deg]
alt0     = 10;
altf     = 61;
speed0   = 15;
speedf   = 24;
gamma0   = 0;
gammaf   = 0;
range0   = 0;

% Minimum and maximum values for state variables
altmin   = 5;
altmax   = 100;
speedmin = 14.013;
speedmax = 27;
gammamin = -40*pi/180;
gammamax = -gammamin;
rangemin = 0;
rangemax = 500;

% Create a trajectory from 50 points to minimize the time to get to cruise
toms t t_f
p = tomPhase('p',t,0,t_f,50);
setPhase(p);

% Initialize the state variables 
tomStates h v gamma range

% Initialize the control variable, angle of attack [deg]
tomControls aalpha

% Supply initial guesses for the algorithm
guess = {
    t_f == 50;
    icollocate({
    h == alt0 + t/t_f*(altf-alt0);
    v == speed0 + t/t_f*(speedf-speed0);
    range == 50;
    gamma == 10*pi/180;
    })};
% Define any constraints
cbox = {
    10 <= t_f <= 100;
    icollocate(altmin <= h <= altmax)
    icollocate(speedmin <= v <= speedmax)
    icollocate(gammamin <= gamma <= gammamax)
    icollocate(rangemin <= range <= rangemax)    
    };
% Define any bounds (initial and final values)
bnd = {
    initial(h) == alt0;
    initial(v) == speed0;
    initial(gamma) == gamma0;
    initial(range) == range0;
    final(h) == altf;
    final(v) == speedf;
    final(gamma) == gammaf;
    };

m       = 4.2;                  % mass [kg]
Clalpha = .09305;               % Cl-Alpha [deg^-1]
CD0     = 0.013;                % Cd0
CL0     = 0.1299;               % Cl0
k       = 0.0538;               % k
S       = 0.275;                % Wing Area [m^2]
g0      = 9.81;                 % Gravity [m/s^2]
rho     = 1.225;                % Density [kg/m^3]
CL      = CL0+Clalpha.*aalpha;  % Coefficient of Lift
CD      = CD0+k.*CL.^2;         % Coefficient of Drag
dynpres = 0.5.*rho.*v.^2;       % Dynamic Pressure [kg/(m-s^2)]
D       = dynpres.*S.*CD;       % Drag [N]
L       = dynpres.*S.*CL;       % Lift [N]
T       = 6.5;                % Maximum Thrust [N] assumed to be constant

% State space equations for the necessary states
equ = collocate({
    dot(h) == v.*sin(gamma);
    dot(v) == ((T.*cos(aalpha)-D-m.*g0.*sin(gamma))./m);
    dot(gamma) == (T.*sin(aalpha)+L-m.*g0.*cos(aalpha))./(m.*v);
    dot(range) == v.*cos(gamma);
});

options = struct;
options.name = 'Minimum Time to Climb (Metric)';
options.scale = 'auto';

% Starting guess of gamma is in confilct with the boundary conditions, but
% that's ok. (It will give a warning, which we suppress.)
warns = warning('off', 'tomSym:x0OutOfBounds');
ezsolve(t_f,{cbox,bnd,equ},guess,options);

% Restore warning
warning(warns);

% Plot the generated state variables
figure(); 
subplot(2,1,1);
ezplot([h, v, gamma*180/pi range])
legend('Altitude [m]','Velocity [m/s]', 'Flight Path Angle [deg]','range');
title('Minimum Time to Climb State Variables');
subplot(2,1,2)
ezplot(aalpha);
title('Minimum Time to Climb Control');
legend('Angle of Attack [deg]')