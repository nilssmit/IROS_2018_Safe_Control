%RUNSIMULATOR Runs an interactive simulator of a compass gait walking model
% with a guaranteed safe semi-autonomous controller.
% Make sure spotless is in path before starting.

%   More information can be found in the paper titled "Guaranteed Safe 
%   Semi-Autonomous Control of Hybrid Systems" submitted to Robotics and 
%   Automation Letters and IROS 2018.

%   Created by Nils Smit-Anseeuw (1) on 2-15-18
%   MATLAB 2017a

%   (1) Robotics and Optimization for Analysis of Human Motion
%       University of Michigan Ann Arbor
%       nilssmit@umich.edu
%% Add local paths (make sure spotless is already on the path)
addpath('SavedData')
addpath('Utils/CompassGait');
addpath('Utils/General');

%% Load guaranteed safe semi-autonomous controller (saved optimization output)
optOut = load('OptimizationOutput.mat');
% uSemiAuton takes in the current state and user input, and returns a 
% control input that is guaranteed to keep the walker safe.
% The barrier function and safe controller used to generate this controller
% are also found in optOut as vSafe and uSafe respectively.
% The semi-autonomous controller takes over when the value of v drops below
% a threshold of 0.2.
uSemiAuton = optOut.uMaskFN_US;

%% Set initial condition for simulator
x0_US = [0.01; 0.6; 0.6; 0];

%% Run interactive simulator
interactiveCompassGait(x0_US,uSemiAuton)
