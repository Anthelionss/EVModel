%%建立s-function
function [ sys,x0,str,ts] = TransmissionModel(t,x,u,flag,r_gear,eff_trans)

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 2
        sys = mdlUpdate(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u,r_gear,eff_trans);
    case 4
        sys = mdlGetTimeOfNextVarHit(t,x,u);
    case 9
        sys = mdlTerminate(t,x,u);                 
    otherwise
        DAStudio.error(['Simulink:block:unhandledflag',num2str(flag)]);
end

%%初始化s-function
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 0;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 2;
sizes.NumInputs = 2;
sizes.DirFeedthrough = 1; 
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [];
str = [];
ts = [0 0];

function sys = mdlDerivatives(~,~,~)
sys = [];

function sys=mdlUpdate(~,~,~)
sys=[];

%%TransmissionModel  输出
function sys=mdlOutputs(~,~,u,r_gear,eff_trans)
Tin = u(1); nout = u(2);
Tout = r_gear * eff_trans * Tin;
nin = r_gear * nout;
sys = [Tout nin];

function sys = mdlGetTimeOfNextVarHit(t,~,~)
sampleTime = 1;
sys = t+sampleTime;

function sys = mdlTerminate(~,~,~)
sys = [];
