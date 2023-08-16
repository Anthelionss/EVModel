%%建立s-function
function [ sys,x0,str,ts] = BatteryModel(t,x,u,flag,SOC_initial,Bat_Cap_Ah)

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes();
    case 1
        sys = mdlDerivatives(t,x,u,SOC_initial,Bat_Cap_Ah);
    case 2
        sys = mdlUpdate(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u,SOC_initial);
    case 4
        sys = mdlGetTimeOfNextVarHit(t,x,u);
    case 9
        sys = mdlTerminate(t,x,u);                 
    otherwise
        DAStudio.error(['Simulink:block:unhandledflag',num2str(flag)]);
end

%%初始化s-function
function [sys,x0,str,ts] = mdlInitializeSizes()
sizes = simsizes;
sizes.NumContStates = 2;
sizes.NumDiscStates = 0;
sizes.NumOutputs = 3;
sizes.NumInputs = 1;
sizes.DirFeedthrough = 1; 
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [0 0];
str = [];
ts = [0 0];

function sys = mdlDerivatives(~,x,u,SOC_initial,Bat_Cap_Ah)
I = u(1);
%%电压-SOC表
X=0.25:0.05:1;
Y=[329.578956912698 331.264061643644 334.222416796755 335.977481552031 ...
   337.118187606370 339.421595672392 340.308085040362 340.898981163811 ...	
   343.018898459522 346.179322896040 346.685239695678 346.505505514473 ...
   348.067447737458 352.646058531939 361.012183734023 365.528165463448];
SOC = x(1) + SOC_initial;
U = interp1(X,Y,SOC);%%插值查表
%%系统微分方程
dSOC = - I / (Bat_Cap_Ah * 3600);
dW = U * I / (3600 * 1000);
sys = [dSOC dW];

function sys=mdlUpdate(~,~,~)
sys=[];

%%输出
function sys=mdlOutputs(~,x,~,SOC_initial)
%%电压-SOC表
X=0.25:0.05:1;
Y=[329.578956912698 331.264061643644 334.222416796755 335.977481552031 ...
   337.118187606370 339.421595672392 340.308085040362 340.898981163811 ...	
   343.018898459522 346.179322896040 346.685239695678 346.505505514473 ...
   348.067447737458 352.646058531939 361.012183734023 365.528165463448];
SOC = x(1) + SOC_initial;
U = interp1(X,Y,SOC);%%插值查表
W = x(2);
sys = [SOC U W];

function sys = mdlGetTimeOfNextVarHit(t,~,~)
sampleTime = 1;
sys = t+sampleTime;

function sys = mdlTerminate(~,~,~)
sys = [];
