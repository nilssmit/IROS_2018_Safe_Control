classdef cgLeg
    properties (SetAccess = 'private', GetAccess = 'public')
        h_fig
        h_foot
        h_ankle
        h_leg
        h_hip
        h_all
        lLeg
        footPatch
        legPatch
        jointRect
        vOffset
    end
    methods
        function obj = cgLeg(figHandle,faceColor,lineWidth)
            if nargin < 2
                edgeColor = [128,0,0]/256;
                faceColor = [211,141,95]/256;
            else
                edgeColor = 0.5*faceColor;
            end
            if nargin < 3
                lineWidth = 3;
            end
            CGparams;
            wFoot = 0.075;
            wLeg = 0.025;
            obj.vOffset = 2*wLeg + 0.005;
            lLeg = lL;
            obj.lLeg = lLeg;
            
            if (nargin < 1) || (isempty(figHandle))
                obj.h_fig = figure('units','inches','position',[0.5,0.5,6,6]);
                clf,hold on, axis equal
                xlim([-0.3,0.3]),ylim([-0.1,0.5])
            else
                obj.h_fig = figHandle;
                figure(figHandle)
            end
            
            obj.footPatch = [wLeg,wFoot,-wFoot,-wLeg,wLeg;
                             0,-2*wLeg,-2*wLeg,0,0];
            
            obj.h_foot = patch(obj.footPatch(1,:),...
                               obj.footPatch(2,:) + obj.vOffset,...
                               faceColor,...
                               'edgecolor',edgeColor,...
                               'linewidth',lineWidth);
                           
            obj.legPatch = [wLeg,wLeg,-wLeg,-wLeg,wLeg;
                            0,lLeg,lLeg,0,0];
            
            obj.h_leg = patch(obj.legPatch(1,:),...
                              obj.legPatch(2,:) + obj.vOffset,...
                              faceColor,...
                              'edgecolor',edgeColor,...
                              'linewidth',lineWidth);
            
            obj.jointRect = [-wLeg,-wLeg,2*wLeg,2*wLeg];
            
            obj.h_ankle = rectangle('position',obj.jointRect + [0,obj.vOffset,0,0],...
                                    'curvature',[1,1],...
                                    'facecolor',faceColor,...
                                    'edgecolor',edgeColor,...
                                    'linewidth',lineWidth);
            
            obj.h_hip = rectangle('position',obj.jointRect + [0,obj.vOffset+lLeg,0,0],...
                                  'curvature',[1,1],...
                                  'facecolor',faceColor,...
                                  'edgecolor',edgeColor,...
                                  'linewidth',lineWidth);
                              
            obj.h_all = [obj.h_hip;
                         obj.h_leg;
                         obj.h_ankle;
                         obj.h_foot];
            
        end
        function update(obj,footPos,angle)
            rotMat = [cos(angle),sin(angle);
                      -sin(angle),cos(angle)];
            
            footPatch_ = obj.footPatch + repmat(footPos(:),1,5);
            
            set(obj.h_foot,'xdata',footPatch_(1,:),'ydata',footPatch_(2,:) + obj.vOffset)
            
            legPatch_ = rotMat*obj.legPatch + repmat(footPos(:),1,5);
            
            set(obj.h_leg,'xdata',legPatch_(1,:),'ydata',legPatch_(2,:) + obj.vOffset)
            
            set(obj.h_ankle,'position',obj.jointRect + [footPos(:)',0,0] + [0,obj.vOffset,0,0])
            
            hipPos_ = footPos(:) + rotMat*[0;obj.lLeg];
            
            set(obj.h_hip,'position',obj.jointRect + [hipPos_',0,0] + [0,obj.vOffset,0,0])
            
            figure(obj.h_fig)
            xlim(hipPos_(1)+[-0.3,0.3])
        end
        function setColor(obj,faceColor)
            edgeColor = 0.5*faceColor;
            set(obj.h_all,'facecolor',faceColor,'edgeColor',edgeColor)
        end
    end
end