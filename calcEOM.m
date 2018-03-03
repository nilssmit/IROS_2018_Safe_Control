function calcEOM
%CALCEOM Generates the equations of motion for a compass gait walker
% This MATLAB function performs the symbolic computation of the equations
% of motion for a compass gait walker with massive legs. The mass and
% length parameters of the robot are stored in the function 'CGparams.m'.
% The output of this function is the file 'SavedData/CGEOM.mat', which 
% contains symbolic expressions for the dynamics, guard and reset map for 
% this model, as well as kinematic properties of the model for animation
% purposes.

%   More information can be found in the paper titled "Guaranteed Safe 
%   Semi-Autonomous Control of Hybrid Systems" submitted to Robotics and 
%   Automation Letters and IROS 2018.

%   Created by Nils Smit-Anseeuw (1) on 2-15-18
%   MATLAB 2017a

%   (1) Robotics and Optimization for Analysis of Human Motion
%       University of Michigan Ann Arbor
%       nilssmit@umich.edu

%% Get the compass gait parameters
CGparams

%% Define variables
syms x_ y alpha_ theta dx dy dalpha dtheta

%% Get continuous dynamics using minimal coordinates
% Minimal coordinates
q_Min = [theta,alpha_]; 
dq_Min = [dtheta,dalpha];

Stance_Leg_Ang_Min = -theta;
Stance_Leg_CoG_Min = [(lL - lCoG)*sin(theta);
                  (lL - lCoG)*cos(theta)];

MB_CoG_Min = [lL*sin(theta);
          lL*cos(theta)];
      
Swing_Leg_Ang_Min = Stance_Leg_Ang_Min + alpha_;
Swing_Leg_CoG_Min = MB_CoG_Min + [lCoG*sin(Swing_Leg_Ang_Min);
                          -lCoG*cos(Swing_Leg_Ang_Min)];
   
d_Stance_Leg_Ang_Min = jacobian(Stance_Leg_Ang_Min,q_Min)*dq_Min.';
d_Swing_Leg_Ang_Min = jacobian(Swing_Leg_Ang_Min,q_Min)*dq_Min.';

d_Stance_Leg_CoG_Min = jacobian(Stance_Leg_CoG_Min,q_Min)*dq_Min.';
d_MB_CoG_Min = jacobian(MB_CoG_Min,q_Min)*dq_Min.';
d_Swing_Leg_CoG_Min = jacobian(Swing_Leg_CoG_Min,q_Min)*dq_Min.';
                      
% Potential Energy
V_Min = Stance_Leg_CoG_Min(2)*mL*g + Swing_Leg_CoG_Min(2)*mL*g;
V_Min = prod(simplify(factor(V_Min)));

% Kinetic Energy:         
T_Min = 0.5 * (mL * sum(d_Stance_Leg_CoG_Min.^2) + ...
           mL * sum(d_Swing_Leg_CoG_Min.^2) + ...
           jL * d_Stance_Leg_Ang_Min^2 + ...
           jL * d_Swing_Leg_Ang_Min^2);
T_Min = prod(simplify(factor(T_Min)));

L_Min = T_Min-V_Min;
% Partial derivatives:
dLdq_Min   = jacobian(L_Min,q_Min).';
dLdqdt_Min = jacobian(L_Min,dq_Min).';

% Compute Mass Matrix:
M_Min = jacobian(dLdqdt_Min,dq_Min);
M_Min = simplify(M_Min);

% Compute the coriolis and gravitational forces:
dL_dqdt_dt_Min = jacobian(dLdqdt_Min,q_Min)*dq_Min.';
f_cg_Min = dLdq_Min - dL_dqdt_dt_Min;
f_cg_Min = simplify(f_cg_Min);

d_dqdt_dt_Min = simplify(M_Min\f_cg_Min);

gU_Min = simplify(M_Min\eye(2));
gD_Min = gU_Min;

% Swing foot position
Swing_Foot_Pos_Min = MB_CoG_Min + [lL*sin(Swing_Leg_Ang_Min);
                                   -lL*cos(Swing_Leg_Ang_Min)];

% Swing foot Jacobian
J_Min = simplify(jacobian(Swing_Foot_Pos_Min,q_Min));

%% Get discrete dynamics using floating body coordinates
% Floating body coordinates
q_FB = [x_,y,theta,alpha_];
dq_FB = [dx,dy,dtheta,dalpha];

MB_CoG_FB = [x_;
          y];

Stance_Leg_Ang_FB = -theta;
Stance_Leg_CoG_FB = MB_CoG_FB + [lCoG*sin(Stance_Leg_Ang_FB);
                          -lCoG*cos(Stance_Leg_Ang_FB)];

Swing_Leg_Ang_FB = Stance_Leg_Ang_FB + alpha_;
Swing_Leg_CoG_FB = MB_CoG_FB + [lCoG*sin(Swing_Leg_Ang_FB);
                          -lCoG*cos(Swing_Leg_Ang_FB)];

d_Stance_Leg_Ang_FB = jacobian(Stance_Leg_Ang_FB,q_FB)*dq_FB.';
d_Swing_Leg_Ang_FB = jacobian(Swing_Leg_Ang_FB,q_FB)*dq_FB.';

d_Stance_Leg_CoG_FB = jacobian(Stance_Leg_CoG_FB,q_FB)*dq_FB.';
d_MB_CoG_FB = jacobian(MB_CoG_FB,q_FB)*dq_FB.';
d_Swing_Leg_CoG_FB = jacobian(Swing_Leg_CoG_FB,q_FB)*dq_FB.';
                      
% Potential Energy
V_FB = Stance_Leg_CoG_FB(2)*mL*g + Swing_Leg_CoG_FB(2)*mL*g;
V_FB = prod(simplify(factor(V_FB)));

% Kinetic Energy:         
T_FB = 0.5 * (mL * sum(d_Stance_Leg_CoG_FB.^2) + ...
           mL * sum(d_Swing_Leg_CoG_FB.^2) + ...
           jL * d_Stance_Leg_Ang_FB^2 + ...
           jL * d_Swing_Leg_Ang_FB^2);
T_FB = prod(simplify(factor(T_FB)));

L_FB = T_FB-V_FB;
% Partial derivatives:
dLdqdt_FB = jacobian(L_FB,dq_FB).';
      
% Compute Mass Matrix:
M_FB = jacobian(dLdqdt_FB,dq_FB);
M_FB = simplify(M_FB);      

% Swing foot position
Swing_Foot_Pos_FB = MB_CoG_FB + [lL*sin(Swing_Leg_Ang_FB);
                           -lL*cos(Swing_Leg_Ang_FB)];

% Swing foot Jacobian
J_FB = simplify(jacobian(Swing_Foot_Pos_FB,q_FB));

% Update map in velocity
dqPlus_FB = simplify((eye(4) - M_FB\(J_FB.'*((J_FB*(M_FB\(J_FB.')))\J_FB)))*dq_FB.');

dqPlus_FB = simplify(subs(dqPlus_FB,alpha_,2*theta));

%% Translate to minimal coordinates
dqPlus_Min = subs(dqPlus_FB,d_MB_CoG_FB,d_MB_CoG_Min);
dqPlus_Min = simplify(dqPlus_Min(3:4));
dqPlus_Min = [dqPlus_Min(1) - dqPlus_Min(2),... % theta plus = theta minus - alpha minus
               - dqPlus_Min(2)]; % alpha plus = -alpha minus

%% Save dynamics and reset map

% Symbolic state vector
x = {[q_Min,dq_Min].'};

% Continuous dynamics
f = {[dq_Min.'
     d_dqdt_dt_Min]};
gU = {[zeros(2);gU_Min]};
gD = {[zeros(2);gD_Min]};

% Guard at foot touchdown
S = {[2*theta - alpha_;
     -(dalpha - 2*dtheta)]};

% Reset at foot touchdown
R = {[theta - alpha_;
      -alpha_;
      dqPlus_Min.']};

% Link positions (for animation purposes)
linkPos = [0, 0;
           MB_CoG_Min.';
           Swing_Foot_Pos_Min.'];


save('SavedData/CGEOM','x','f','gU','gD','S','R','linkPos')

end
