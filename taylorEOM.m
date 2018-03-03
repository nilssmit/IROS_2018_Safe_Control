function taylorEOM
%TAYLOREOM Taylor expands the equations of motion for a compass gait walker
% This MATLAB function first scales the domains of the walker to be unit
% hypercubes (to help with polynomial conditioning). It then computes a 5th
% order taylor expansion of the continuous dynamics, and a 2nd order taylor
% expansion of the reset map at touchdown.
% The output of this function is the file 'SavedData/CGEOM_Taylor.mat',  
% which contains msspoly objects representing the taylor expanded dynamics,
% guards, and reset maps.
% Make sure spotless is in path before starting.

%   More information can be found in the paper titled "Guaranteed Safe 
%   Semi-Autonomous Control of Hybrid Systems" submitted to Robotics and 
%   Automation Letters and IROS 2018.

%   Created by Nils Smit-Anseeuw (1) on 2-15-18
%   MATLAB 2017a

%   (1) Robotics and Optimization for Analysis of Human Motion
%       University of Michigan Ann Arbor
%       nilssmit@umich.edu

%% Add necessary paths (make sure spotless is already on the path)
addpath('SavedData')
addpath('Utils/CompassGait');
addpath('Utils/General');

%% Set domain size constants
TthetaMin = -1; % Min ankle torque
TthetaMax = 1; % Max ankle torque
TalphaMax = 10; % Max hip torque
DthetaMax = 0.1;  % Max ankle disturbance
DalphaMax = 1;  % Max hip torque disturbance

thetaMax = 30*pi/180; % Maximum absolute ankle angle 
alphaMax = 60*pi/180; % Maximum absolute hip angle
vThetaMax = 5; % Maximum ankle velocity
vAlphaMax = 10; % Maximum hip velocity

nModes = 2; % Number of modes

%% Define the ranges for scaling and taylor expansion
thetaLim = linspace(-thetaMax,thetaMax,nModes+1);

x_US_Range = {[ -thetaMax,  0;            % Stance leg angle
                -alphaMax,  alphaMax;     % Between leg angle
                0,          vThetaMax;    % Stance leg velocity (Must always move forward)
                -vAlphaMax, vAlphaMax];   % Between leg velocity
              [ 0,          thetaMax;     % Stance leg angle
                0,          alphaMax;     % Between leg angle
                0,          vThetaMax;    % Stance leg velocity (Must always move forward)
                -vAlphaMax, vAlphaMax]};  % Between leg velocity

x0_Taylor_US = {mean(x_US_Range{1},2);
                mean(x_US_Range{2},2)};

u_US_Range = repcell({[ TthetaMin, TthetaMax;         % Ankle leg torque
                        -TalphaMax, TalphaMax]},nModes,1);  % Hip torque

d_US_Range = repcell({[ -DthetaMax, DthetaMax;         % Ankle leg torque
                        -DalphaMax, DalphaMax]},nModes,1);  % Hip torque
%% Set up dynamics
EOMstruct = load('CGEOM.mat');

x_US_Sym = repcell(EOMstruct.x,nModes,1);
theta_US_Sym = x_US_Sym{1}(1);

f_US = repcell(EOMstruct.f,nModes,1);
gU_US = repcell(EOMstruct.gU,nModes,1);
gD_US = repcell(EOMstruct.gD,nModes,1);

S_US = repcell({[]},nModes,nModes);
R_US = repcell({[]},nModes,nModes);

S_US{1,2} = theta_US_Sym;
R_US{1,2} = x_US_Sym{1};
S_US{2,1} = EOMstruct.S{1};
R_US{2,1} = EOMstruct.R{1};

%% Scale dynamics and Taylor expand 
% Define the msspoly object representing the state
x = {msspoly('xa',4);
     msspoly('xa',4)};

degTaylor = struct('f',5,'gU',5,'gD',5,'S',2,'R',2);
[f, gU, ~, S, R, scaleParam] = scaleTaylorDynamicsHybrid(x_US_Sym, x,...
            f_US, gU_US, gD_US, S_US, R_US, ...
            x_US_Range, u_US_Range, d_US_Range, degTaylor, x0_Taylor_US);

%% Save all generated objects
save('SavedData/CGEOM_Taylor','x','f','gU','S','R','scaleParam')

end