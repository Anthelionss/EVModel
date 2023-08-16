%%建立s-function
function [ sys,x0,str,ts] = VehicleModel(t,x,u,flag,r_Wheel,A_Drag,f0,g,m)

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 1
        sys = mdlDerivatives(t,x,u,r_Wheel,A_Drag,f0,g,m);
    case 2
        sys = mdlUpdate(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u,r_Wheel);
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
sizes.NumContStates = 1;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 3;
sizes.NumInputs = 1;
sizes.DirFeedthrough = 1; 
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = 0;
str = [];
ts = [0 0];

%%VehicleModel
function sys = mdlDerivatives(~,x,u,r_Wheel,A_Drag,f0,g,m)
T = u(1); v_ms = x(1);%速度(m/s)
F_r = f0 * g * m;%计算滚动阻力
F_w = (((v_ms * 3.6)*(v_ms * 3.6)) / 21.15) * A_Drag;%计算风阻
dv_ms = (T/r_Wheel - F_r - F_w)/m;%系统动力学方程
sys = dv_ms;

function sys=mdlUpdate(~,~,~)
sys=[];

%%输出
function sys=mdlOutputs(~,x,~,r_Wheel)
v_ms = x(1);
w = v_ms/r_Wheel * pi/30;%轮角速度
v_kmh = 3.6 * v_ms;%速度(km/h)
sys=[v_ms v_kmh w];

function sys = mdlGetTimeOfNextVarHit(t,~,~)
sampleTime = 1;
sys = t+sampleTime;

function sys = mdlTerminate(~,~,~)
sys=[];
