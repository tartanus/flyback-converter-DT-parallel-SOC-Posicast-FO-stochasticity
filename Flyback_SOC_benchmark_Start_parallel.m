%% %%%%%%%%%%% Flyback Controller Tunning Parameters %%%%%%%%%%%%%%%
% Define PID gains
k1 = 7.51277596604741e-09;        %10 100 50
k2 = 1e5;
% k3 = 1.4220011785462e-06;

% Define posicast gains
delta=0.7;
td=24e-6;
% k1 = 7.51277596604741e-1;        %10 100 50
% k2 = 1e4;
% % k3 = 1.4220011785462e-06;
% 
% % Define posicast gains
% delta=0.1;
% td=24e-7;
%% %%%%%%%%%%% SOC parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

maxKp=1e-3; minKp=1e-6;            %maxKp=1e-3; minKp=1e-6;
maxKi=8e5; minKi=8e4;              %maxKi=1e5; minKi=8e4;  %oldMin=0.01: %
maxKd=0.1; minKd=0.001;            %maxKd=0.1; minKd=0.001;

maxLimDelta=1.1; minLimDelta=0.9;  %old min=0.1; 0.9 and 1.1 for parametric
maxLimTd=1e-5; minLimTd=1e-7;

%Globalized Constrained Nelder Mead GCNM  parameters
GCNMEnable=1;
optimPeriod=1/1000;        %optimization execution period (s)
resetThreshold=0.1;        %probabilistic restart threshold
refSignalPeriod=1/1000;    %Reference signal period
amp=1;                     %Reference amplitude

%constraints limits
OVLim=1;                   %5% overshoot % 1 for parametric 
TSLim=65;                  %Settling time limit (us)
L=1;                       %delay value L=0.1,1,10;
N=100;                     %derivative filter N (Zero for PI controller)

%cost function weights
W1 = 0.1;
W2 = 1;
W3 = 1;


%% Flyback models
%% Model 1
Rsw=0.01;   % 0.01ohms
n=2;        % 9
Lm=1e-6;    % 1e-6 or 9e-6
C=100e-6;
Rc=0;
R=30;       % 30 is Original load
D=20;
Io=5;
Vo=5;

Vs=[24 0];   % 5V
X=[Io Vo];

A1=[-Rsw/Lm 0;
    0 -1/R*C+Rc];
A2=[n^2*Rc*R/(R*Lm-Rc*Lm) n*R/(R*Lm-Rc*Lm);
    -n*R/(R*C-Rc*C) -1/R*C+Rc*C];
B1=[-1/Lm 0;
    0 0];
B2=[0 -n/Lm;
    0 0];
C1=[n*R*Rc/(R-Rc) R/(R-Rc)];
C2=[0 R/(R+Rc)];

A=(A1-A2).*D + A2;
B=(B1-B2).*D + B2;
Ct=(C1'-C2').*D + C2';   % Ct is C transpose

s=tf('s');
TF1_x_d=(s*eye(2)-A)^-1*((A1-A2)*X' + (B1-B2)*Vs');
TF2_y_d=Ct'*(s*eye(2)-A)^-1*((A1-A2)*X' + (B1-B2)*Vs') + (C1'-C2')*X;
TF2_y_d1=Ct'*(s*eye(2)-A)^-1*((A1-A2)*X' + (B1-B2)*Vs');    % (C1'-C2')*X is a zero matrix
[num,den]=tfdata(TF2_y_d1);

Gs11=TF2_y_d1;
flybackTF1=-Gs11;
flybackTFDiscrete1=c2d(flybackTF1,1e-6,'tustin');

%% Model 2
Rsw=0.01;   % 0.01ohms
n=2;        % 9
Lm=1e-6;    % 1e-6 or 9e-6
C=200e-6;
Rc=0;
R=30;       % 30 is Original load
D=20;
Io=5;
Vo=5;

Vs=[24 0];   % 5V
X=[Io Vo];

A1=[-Rsw/Lm 0;
    0 -1/R*C+Rc];
A2=[n^2*Rc*R/(R*Lm-Rc*Lm) n*R/(R*Lm-Rc*Lm);
    -n*R/(R*C-Rc*C) -1/R*C+Rc*C];
B1=[-1/Lm 0;
    0 0];
B2=[0 -n/Lm;
    0 0];
C1=[n*R*Rc/(R-Rc) R/(R-Rc)];
C2=[0 R/(R+Rc)];

A=(A1-A2).*D + A2;
Ba=(B1-B2).*D + B2;
Ct=(C1'-C2').*D + C2';   % Ct is C transpose

s=tf('s');
TF1a_x_d=(s*eye(2)-A)^-1*((A1-A2)*X' + (B1-B2)*Vs');
TF2a_y_d=Ct'*(s*eye(2)-A)^-1*((A1-A2)*X' + (B1-B2)*Vs') + (C1'-C2')*X;
TF2a_y_d1=Ct'*(s*eye(2)-A)^-1*((A1-A2)*X' + (B1-B2)*Vs');    % (C1'-C2')*X is a zero matrix
[numa,dena]=tfdata(TF2a_y_d1);

Gs11a=TF2a_y_d1;
flybackTF2=-Gs11a;
flybackTFDiscrete2=c2d(flybackTF2,1e-6,'tustin');

%% Model 3
Rsw=0.01;   % 0.01ohms
n=2;        % 9
Lm=1e-6;    % 1e-6 or 9e-6
C=300e-6;
Rc=0;
R=30;       % 30 is Original load
D=20;
Io=5;
Vo=5;

Vs=[24 0];   % 5V
X=[Io Vo];

A1=[-Rsw/Lm 0;
    0 -1/R*C+Rc];
A2=[n^2*Rc*R/(R*Lm-Rc*Lm) n*R/(R*Lm-Rc*Lm);
    -n*R/(R*C-Rc*C) -1/R*C+Rc*C];
B1=[-1/Lm 0;
    0 0];
B2=[0 -n/Lm;
    0 0];
C1=[n*R*Rc/(R-Rc) R/(R-Rc)];
C2=[0 R/(R+Rc)];

A=(A1-A2).*D + A2;
Bb=(B1-B2).*D + B2;
Ct=(C1'-C2').*D + C2';   % Ct is C transpose

s=tf('s');
TF1b_x_d=(s*eye(2)-A)^-1*((A1-A2)*X' + (B1-B2)*Vs');
TF2b_y_d=Ct'*(s*eye(2)-A)^-1*((A1-A2)*X' + (B1-B2)*Vs') + (C1'-C2')*X;
TF2b_y_d1=Ct'*(s*eye(2)-A)^-1*((A1-A2)*X' + (B1-B2)*Vs');    % (C1'-C2')*X is a zero matrix
[numb,denb]=tfdata(TF2b_y_d1);

Gs11b=TF2b_y_d1;
flybackTF3=-Gs11b;
flybackTFDiscrete3=c2d(flybackTF3,1e-6,'tustin');


%% fractional-order randomness
alphaFoPar=0.5;
repetitions=30;
pathName="SOCFOParallel\";
% alpha=1.5;
for k=1:repetitions
    for alpha=0.1:0.1:1
    %generate fractional order noise for the GCNM probabilistic restart
    alphaFoPar=alpha;
%     timeVec=0:optimPeriod:tSim;
%     timeVecFinal=0:ts:tSim;
%     kpRandAux=alphaStableNoise(alpha,1000)*maxKp;
%     kiRandAux=alphaStableNoise(alpha,1000)*maxKi;
%     randNoiseAux=alphaStableNoise(alpha,1000);
%     kpRand=[]; kiRand=[]; randNoise=[];
%     tic
%     for j=1:length(timeVec)
%         kpRand=[kpRand; ones(optimPeriod/ts,1)*kpRandAux(j)];
%         kiRand=[kiRand; ones(optimPeriod/ts,1)*kiRandAux(j)];
% %         randNoise=[randNoise ones(optimPeriod/ts,1)*randNoiseAux(j)];
%     end
%     toc
%     
%     %random initial condition vector for the simulations
%     randomKp=[timeVecFinal' kpRand(1:length(timeVecFinal))];
%     randomKi=[timeVecFinal' kiRand(1:length(timeVecFinal))];
% %     randomNoiseOutput=[timeVecFinal' randNoise(1:length(timeVecFinal))];
    tic
    % Run the Benchmark file
    sim("ICCMA_posicast_23_parallelGCNM_FO.slx")%,'SrcWorkspace','current');
    SOCParallelAnalytics
    execTimeIter=toc
    %saving data
    tic
    fileName=strcat(pathName,string(k),'alpha',string(alpha*10),'.mat');
    save(fileName)
    toc
    end
end



