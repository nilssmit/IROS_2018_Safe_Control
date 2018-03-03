function [uMaskUS, v, z] = uMaskUSFN(xUS,uUS,uMaskFN_SC,thetaLim,scaleParam)
thetaLim = [-inf,thetaLim(2:end-1),inf];

mode = find(xUS(1) < thetaLim,1)-1;

xSC = scaleParam.x_SC_Fun{mode}(xUS);
uSC = scaleParam.u_SC_Fun{mode}(uUS);

[uMaskSC,v,z] = uMaskFN_SC{mode}(xSC,uSC);

uMaskUS = scaleParam.u_US_Fun{mode}(uMaskSC);

end
