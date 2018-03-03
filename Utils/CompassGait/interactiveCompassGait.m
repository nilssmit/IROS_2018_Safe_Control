function interactiveCompassGait(x0,uSemiAuton,recVid)
%INTERACTIVECOMPASSGAIT Interactive simulator of a compass gait walker. 
% This MATLAB function simulates a compass gait walker using trigonometric
% dynamics and taylor expanded reset map.
% There are two modes to the simulator: Raw Mode, in which user input is
% sent directly to the walker, and Semi-autonomous Mode, in which the user
% input is fed through the provided semi-autonomous controller before being
% sent to the walker.
% The value of the barrier function used by the semi-autonomous controller
% is plotted alongside the model, and the controls are shown on-screen.
%
% Input:  - x0: The initial state of the robot (4x1 vector)
%         - uSemiAuton: The semi-autonomous controller. This is a function
%                       that takes as input, the state and current
%                       controller, and returns a modified input, the
%                       current value of the barrier function, and the
%                       current weighting factor of the user input.
%         - recVid: (optional) Set this to 1 to save a video of the
%                   simulator output.

%   More information can be found in the paper titled "Guaranteed Safe 
%   Semi-Autonomous Control of Hybrid Systems" submitted to Robotics and 
%   Automation Letters and IROS 2018.

%   Created by Nils Smit-Anseeuw (1) on 2-15-18
%   MATLAB 2017a

%   (1) Robotics and Optimization for Analysis of Human Motion
%       University of Michigan Ann Arbor
%       nilssmit@umich.edu

%% Set constants
% If recVid is unspecified, default to 0 (no video)
if nargin < 3
    recVid = 0;
elseif recVid
% Otherwise, if recVid is 1, initialize the video file
    date = datestr(now,'yyyy_mm_dd_T_HH_MM_SS');
    vidObj = VideoWriter(['..\..\VideoOutput\interactiveRec' date], 'MPEG-4');
    vidObj.Quality = 100;
    frames = [];
    open(vidObj)
end

% slowDown specifies the simulation speed. 
slowDown = 3; % a value of 3 means we simulate at 1/3X real-time

% Where the v threshold is shown on the plot
vMPlot = 0.2;

% Set to 1 in order to show the v plot when the semi-autonomous controller
% is inactive.
vOnRaw = 0;

% Set the color scheme of the plotting
cViaEdge = [115, 231, 57]/256;
cViaInt = [76, 187, 23]/256;
cRawTraj = [134, 23, 187]/256;
cSemiTraj = [23, 76, 187]/256;

% Set the time-step of the simulator in sec (this is the rate at which the
% simulator checks for new user input).
dt = 0.02; 

%% Load dynamics and kinematics functions from file
% Load the exact dynamics (trigonometric, used for the continuous dynamics)
EOMstruct = load('CGEOM.mat');
% Load the taylor expanded dynamics (polynomial, used for the reset map)
taylorEOMstruct = load('CGEOM_Taylor.mat');

% Extract the continuous dynamics and define as matlab functions
fFN = matlabFunction(EOMstruct.f{1},'vars',EOMstruct.x);
gUFN = matlabFunction(EOMstruct.gU{1},'vars',EOMstruct.x);
%   Extract the guard
SFN = matlabFunction([EOMstruct.S{1};EOMstruct.x{1}(1)],'vars',EOMstruct.x);
%   Set the model domain (Note: this is wider than the domain in the paper. 
%   We do this for ease of user control. The semi-autonomous controller will
%   still stay within the domain specified in the paper)
hXFN = @(x_) [(x_(1)+pi/2)*(-x_(1) + pi/2);
             (x_(2)+pi)*(-x_(2) + pi);];

% Extract scaled and taylor expanded variables
%   Extract the free variable of the msspolys
x = taylorEOMstruct.x;
%   Get the structure with all scaling information 
scaleParam = taylorEOMstruct.scaleParam;
%   Get the scaled, taylor expanded reset map
R_Taylor = taylorEOMstruct.R;
%   Unscale the reset map
RFN = fn(scaleParam.x_US_Fun{1}(subs(R_Taylor{2,1},x{2},scaleParam.x_SC_Fun{2}(x{2}))),x{2});
%   Unscale user input from [-1,1] to [u_min, u_max]
uUSFN = scaleParam.u_US_Fun{1}; 

% Define the kinematics as a matlab function (for plotting)
linkPosFN = matlabFunction(EOMstruct.linkPos,'vars',EOMstruct.x);

%% Initialize global variables 
% compass gait state
cgState = x0;

% Offset in x (increased on each touchdown)
xMod = 0;

% Foot swap (for plotting, toggled on touchdown)
switchFoot = 0;

% Initial user input
uInGlobal = [0,0]';

% Simulator pause flag
pauseGlobal = 0;

% Mask flag: 0 => Raw Mode, 1 => semi-autonomous mode
modeFlagGlobal = 1; % Start in semi-autonomous mode

% v storage array for plot
vStor = zeros(100,1);
tStor = vStor;

%% Draw everything
% Draw compass gait figure
h_fig = figure('units','inches','position',[0.5,0.5,9,6],...
               'keypressfcn',@(~,evt) kpfun(evt),...
               'keyReleaseFcn',@(~,evt) krfun(evt));
clf,hold on, axis equal
xlim([-0.3,0.3]),ylim([-0.05,0.5])
hAxCG = gca;
set(hAxCG,'position',[-0.07 0.12 0.82 0.82],...
'yticklabel','none','ytick',[],'ycolor','w','fontsize',14)
xlabel('Distance [m]','fontsize',16)

% Each leg is defined wrt the foot position
legL = cgLeg(h_fig,cSemiTraj);
legR = cgLeg(h_fig,cSemiTraj);
linkPos_ = linkPosFN(cgState);
legL.update(linkPos_(1,:),cgState(1));
legR.update(linkPos_(3,:),cgState(1)-cgState(2));
hGD = line([-0.3,0.3],[0,0],'color','k','linewidth',3);

% Draw v plot figure
t_window = 1;
hAxv = axes('position',[0.69,0.7,0.3,0.24],'xtick',[],'xticklabel',[],'ytick',[0,vMPlot,1]);
hAxAll = hAxv;
title('Margin of Safety (v(x))')
xlim([-t_window,0]),ylim([0,1]),hold on
hAxAll(end+1) = rectangle('position',[-t_window,0,1e4,1],'facecolor',cViaInt,'edgecolor','none');
hAxAll(end+1) = rectangle('position',[-t_window,0,1e4,vMPlot],'facecolor',cViaEdge,'edgecolor','none');
set(hAxv ,'Layer', 'Top')
hLv = plot(tStor,vStor,'color',cSemiTraj,'linewidth',2);
hAxAll(end+1) = hLv;

% Draw Controller Boxes
hSafe = annotation('textbox',[0.69 0.33 0.3 0.1],'String','Safe Controller',...
           'VerticalAlignment','middle','HorizontalAlignment','center',...
           'FontSize',16,'linewidth',3,...
           'EdgeColor',0.5*cViaInt,'BackgroundColor',cViaInt);
hRaw = annotation('textbox',[0.69 0.50 0.3 0.1],'String','User Input',...
           'VerticalAlignment','middle','HorizontalAlignment','center',...
           'FontSize',16,'linewidth',3,...
           'EdgeColor',0.5*cRawTraj,'BackgroundColor',cRawTraj);

% Draw user instructions
annotation('textbox',[0.69 0.08 0.3 0.16],'String',...
           {'Controls',...
            'Q: +ve ankle torque, W: -ve ankle torque',...
            'O: +ve hip torque,     P: -ve hip torque',...
            'M: toggle mode (raw/semi-autonomous)',...
            'S: stop simulator,      R: reset simulator' })

drawnow

%% Set callback functions
% Key-press function is called anytime a key is pressed
function kpfun(evt)
    switch evt.Key
        case 'q' % Negative user torque on ankle
            uInGlobal(1) = -1;
        case 'w' % Positive user torque on ankle 
            uInGlobal(1) = 1;
        case 'o' % Negative user torque on hip
            uInGlobal(2) = -1;
        case 'p' % Positive user torque on hip
            uInGlobal(2) = 1;
        case 'm' % Toggle mode between raw and semi-autonomous
            modeFlagGlobal = ~modeFlagGlobal;
            if modeFlagGlobal
                set(hAxAll,'visible','on')
                set(hSafe,'visible','on')
                legL.setColor(cSemiTraj);
                legR.setColor(cSemiTraj);
                set(hLv,'color',cSemiTraj)
            else
                if ~vOnRaw
                    set(hAxAll,'visible','off')
                    set(hSafe,'visible','off')
                end
                legL.setColor(cRawTraj);
                legR.setColor(cRawTraj);
                set(hLv,'color',cRawTraj)
                set(hSafe,'backgroundcolor','w')
            end
        case 's' % Stop simulation
            pauseGlobal = 1;

            if recVid
                writeVideo(vidObj,frames);
                close(vidObj);
            end
        case 'r' % Reset simulation to initial state
            cgState = x0;
            modeFlagGlobal = 1;
            pauseGlobal = 0;
            legL.setColor(cSemiTraj);
            legR.setColor(cSemiTraj);
            set(hLv,'color',cSemiTraj)
            set(hAxAll,'visible','on')
            set(hSafe,'visible','on')
            walk;
    end
end

% Key-release function is triggered any time a key is released
function krfun(evt)
    switch evt.Key
        case {'q','w'} % Ankle input is released
            uInGlobal(1) = 0;
        case {'o','p'} % Hip input is released
            uInGlobal(2) = 0;
    end
end

tic
walk

% Walk function runs continuously while the walker is moving
function walk
    while ~pauseGlobal
        %% Update car state
        % Unscale the user input from [-1,1] to [u_min,u_max]
        u0 = uUSFN(uInGlobal);
        
        if modeFlagGlobal % Modify the input if in semi-autonomous mode
            u = @(x_) uSemiAuton(x_,u0);
        else % Keep user input if in raw mode
            u = @(x_) [u0(1);u0(2)/4]; % we scale down the user hip input for ease of control
        end
        % Define the resulting dynamics
        odeFN = @(x) fFN(x) + gUFN(x)*u(x);

        % Simulate the dynamics under this input for dt seconds
        [tTraj,xTraj,~,exitFlag] = simulateHybrid({odeFN}, {SFN}, {RFN}, cgState, 1, dt, {hXFN}, {[]}, 4,1e-3);

        if exitFlag < 1 % If the robot leaves the domain
            pauseGlobal = 1; % Pause the simulation
            cgState = xTraj{1}(end,:).';
            return
        else % If not, continue
            % Find any events so we can update the state for plotting
            evInds = [find(diff(xTraj{1}(:,1))<-0.1);length(tTraj)];
            for i = 1:length(evInds)-1
                linkPos_ = linkPosFN(xTraj{1}(evInds(i),:)');
                % For each touchdown, we need to chift the robot x position
                xMod = xMod + linkPos_(end,1);
                % For each touchdown we need to toggle the stance foot
                switchFoot = ~switchFoot;
            end
            cgState = xTraj{1}(end,:).';
        end

        %% Update graphics
        linkPos_ = linkPosFN(cgState);
        linkPos_(:,1) = linkPos_(:,1) + xMod;
        set(hGD,'xdata',[-0.3,0.3]+linkPos_(2,1))
        xlim(hAxCG,[-0.3,0.3]+linkPos_(2,1))

        if ~switchFoot
            legL.update(linkPos_(1,:),cgState(1));
            legR.update(linkPos_(3,:),cgState(1)-cgState(2));
        else
            legR.update(linkPos_(1,:),cgState(1));
            legL.update(linkPos_(3,:),cgState(1)-cgState(2));
        end

        %% Update v plot and box colors
        if vOnRaw || modeFlagGlobal
            [~,vVal,zVal] =  uSemiAuton(cgState,u0);
            tStor = [tStor(2:end);tStor(end) + tTraj(end)];
            vStor = [vStor(2:end);vVal];

            initInd = find(vStor,1);

            set(hLv,'xdata',tStor(initInd:end),'ydata',vStor(initInd:end))
            xlim(hAxv,tStor(end) + [-t_window,0])
        end
        if modeFlagGlobal
            set(hSafe,'backgroundcolor',1 - (1-zVal)*(1 - cViaInt))
            set(hRaw,'backgroundcolor',1 - (zVal)*(1 - cRawTraj))
        end
        %% Wait until time to continue
        drawnow
        while toc < slowDown*dt
            drawnow
        end

        tic
        %% Add frames to video
        if recVid
            if isempty(frames)
                frames = getframe(h_fig);
            else
                frames(1,end+1) = getframe(h_fig);
            end
        end
    end
end
    
end 