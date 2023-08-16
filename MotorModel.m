%%建立s-function
function [ sys,x0,str,ts] = MotorModel(t,x,u,flag)

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u);
    case 2
        sys = mdlUpdate(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
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
sizes.NumOutputs = 1;
sizes.NumInputs = 3;
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

%%MotorModel  输出
function sys=mdlOutputs(~,~,u)
T = u(1); n = u(2); U = u(3);
P = T*n/9550;%电机功率计算
I = 1000*P/U;%电机电流计算
sys=I;

function sys = mdlGetTimeOfNextVarHit(t,~,~)
sampleTime = 1;
sys = t+sampleTime;

function sys = mdlTerminate(~,~,~)
sys = [];

