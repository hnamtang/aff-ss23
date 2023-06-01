% Design a V-tracker
clear all; close all; clc

load ./Flying-Lab-data-Linear-Models-20230504/Lin_Model_V_12_gam_0.mat

% Extract ss for V-controller with elevator
%G=G_long([1 3],1);
%G=G_long(:,1);
G=G_long;

if rank(ctrb(G.A,G.B)) == rank(G.A)
    disp('System controllable')
end


%% Setup Low Pass Filters
Tq=.05;  % time constant of LP filter (check again!)
A_lp=-1/Tq;
B_lp=zeros(1,size(G.C,1)); B_lp(1)=1/Tq;
C_lp=zeros(size(G.C,1),1); C_lp(1)=1;
D_lp=eye(size(G.C,1)); D_lp(1,1)=0;
lp=ss(A_lp,B_lp,C_lp,D_lp);
lp.InputName=G.OutputName;
lp.OutputName=G.OutputName; lp.OutputName(1)={'q_{lp}'};
lp.StateName={'x_{qlp}'};

sys=series(G,lp);
[A,B,C,D]=ssdata(sys);


%% Improve Short-Period Damping Ration using Root Locus (SAS)
%rlocus(-sys(1,1)); grid on;  % positive feedback
k_qtheta=.14;  % SP damping approx. 0.478

% Close the pitch rate loop
Acl=A+B(:,1)*k_qtheta*C(1,:);
syscl=ss(Acl,B,C,D);
syscl.InputName=sys.InputName;
syscl.OutputName=sys.OutputName;
syscl.StateName=sys.StateName;

[Acl,Bcl,Ccl,Dcl]=ssdata(syscl);


%% Design V-Controller on Top of SAS
% Performance output
H=C(3,:);  % \delta V_A
Hext=[H 0];  % add state error e_V_A

% PI compensator, output: integral and proportional
F=0;
G=1;
D=[1;0];
J=[0;1];

% Plant + compensator
% - states: x_{qlp}, q, alpha, V_A, Theta, e_V_A
% - outputs: q_{lp}, alpha, V_A, Theta, gamma, GS, integral, proportional
% Augmented dynamics
Aaug=[Acl zeros(size(Acl,1),1);
      -G*H F];
Baug=[B;zeros(1,size(B,2))];
Gaug=[zeros(size(Acl,1),1);G];

% Augmented output
Caug=[C zeros(size(C,1),1);
      -J*H D];
Faug=[zeros(size(C,1),1);J];







% %% Design V-Controller on Top of SAS
% % Extend the state space with state error e_V_A
% Aext=[Acl zeros(size(A,1),1);
%       -C(3,:) 0];
% Bext=[B;zeros(1,size(B,2))];
% Cext=[C zeros(size(C,1),1)];
% Dext=D;
% 
% % Only P compensator
