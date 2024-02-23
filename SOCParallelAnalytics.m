%% plot benchmark data
close all
time=benchData_PID.Time;

kpOptim=benchData_PID.Data(:,1); 
kiOptim=benchData_PID.Data(:,2);
costJ=benchData_PID.Data(:,3);
ref=benchData_PID.Data(:,4);
error=benchData_PID.Data(:,5);
yOut=benchData_PID.Data(:,6);
OVOptim=benchData_PID.Data(:,7);
TSOptim=benchData_PID.Data(:,8);
yIC=benchData_PID.Data(:,9);     %response with IC only

Ts=1e-6;
pulsePeriod=1/1e3;

figure(1)
subplot(2,1,1)
plot(time,kpOptim)
title('Optimal kp');xlabel('Time (s)'); ylabel('Kp')
set(gca,'FontSize', 14);
subplot(2,1,2)
plot(time,kiOptim)
title('Optimal ki');xlabel('Time (s)'); ylabel('Ki')
set(gca,'FontSize', 14);

figure(2)
subplot(2,3,1:3)
plot(time,yOut)
ylim([-2,2])
hold on
plot(time,ref)
set(gca,'FontSize', 20);
legend('y GCNM','Reference')
title('Optimal y');xlabel('Time (s)'); ylabel('y')


subplot(2,3,4)
plot(time(pulsePeriod/2/Ts:pulsePeriod/Ts),yOut(pulsePeriod/2/Ts:pulsePeriod/Ts))
hold on
% plot(time(pulsePeriod/2/Ts:600/Ts),yIC(pulsePeriod/2/0.01:600/Ts))
plot(time(pulsePeriod/2/Ts:pulsePeriod/Ts),ref(pulsePeriod/2/Ts:pulsePeriod/Ts))
set(gca,'FontSize', 20);
legend('y GCNM','y IC')
title('y from pulsePeriod/2 to 600s');xlabel('Time (s)'); ylabel('y')

subplot(2,3,5)
plot(time(50*pulsePeriod/2/Ts:50*pulsePeriod/2/Ts+pulsePeriod/Ts),yOut(50*pulsePeriod/2/Ts:50*pulsePeriod/2/Ts+pulsePeriod/Ts))
hold on
% plot(time(pulsePeriod/2/Ts:600/Ts),yIC(pulsePeriod/2/0.01:600/Ts))
plot(time(50*pulsePeriod/2/Ts:50*pulsePeriod/2/Ts+pulsePeriod/Ts),ref(50*pulsePeriod/2/Ts:50*pulsePeriod/2/Ts+pulsePeriod/Ts))
set(gca,'FontSize', 20);
legend('y GCNM','y IC')
title('y from pulsePeriod/2 to 600s');xlabel('Time (s)'); ylabel('y')

subplot(2,3,6)
plot(time(90*pulsePeriod/2/Ts:90*pulsePeriod/2/Ts+pulsePeriod/Ts),yOut(90*pulsePeriod/2/Ts:90*pulsePeriod/2/Ts+pulsePeriod/Ts))
hold on
% plot(time(pulsePeriod/2/Ts:600/Ts),yIC(pulsePeriod/2/0.01:600/Ts))
plot(time(90*pulsePeriod/2/Ts:90*pulsePeriod/2/Ts+pulsePeriod/Ts),ref(90*pulsePeriod/2/Ts:90*pulsePeriod/2/Ts+pulsePeriod/Ts))
set(gca,'FontSize', 20);
legend('y GCNM','y IC')
title('y from pulsePeriod/2 to 600s');xlabel('Time (s)'); ylabel('y')


% 
% subplot(2,3,5)
% plot(time(27000/Ts:pulsePeriod/2/Ts),yOut(27000/Ts:27pulsePeriod/2/Ts))
% hold on
% % plot(time(27000/Ts:27pulsePeriod/2/Ts),yIC(27000/Ts:27pulsePeriod/2/Ts))
% plot(time(27000/Ts:pulsePeriod/2/Ts),ref(27000/Ts:pulsePeriod/2/Ts))
% set(gca,'FontSize', 20);
% legend('y GCNM','y IC')
% title('y from 27000 to 27pulsePeriod/2s');xlabel('Time (s)'); ylabel('y')


% subplot(2,3,6)
% plot(time(pulsePeriod/2/Ts:45pulsePeriod/2/Ts),yOut(45000/Ts:pulsePeriod/Ts))
% hold on
% plot(time(45000/Ts:45pulsePeriod/2/Ts),yIC(45000/Ts:45pulsePeriod/2/Ts))
% % plot(time(45000/Ts:45pulsePeriod/2/Ts),ref(45000/Ts:45pulsePeriod/2/Ts))
% set(gca,'FontSize', 20);
% legend('y GCNM','y IC')
% title('y from 45000 to 45pulsePeriod/2s');xlabel('Time (s)'); ylabel('y')

figure(3)
subplot(2,1,1)
plot(time,OVOptim)
ylim([0,100])
set(gca,'FontSize', 14);
hold on
title('Optimal OV');xlabel('Time (s)'); ylabel('Overshoot')
subplot(2,1,2)
plot(time,TSOptim)
set(gca,'FontSize', 14);
title('Settling time');xlabel('Time (s)'); ylabel('Settling time (s)')

figure(4)
counter=1;
clear costJSub timeSub
for i=1:pulsePeriod/Ts:length(costJ)-pulsePeriod/Ts
%     if i==1
%         index=6;
%     else
%         index=i;
%     end
%     
    costJSub(counter)=max(costJ(i:i+pulsePeriod/2/Ts));
    timeSub(counter)=max(time(i:i+pulsePeriod/2/Ts));
    counter=counter+1;
end

% reducedSampleFactor=600;
% costJSub=downsample(costJ,reducedSampleFactor)
% timeSub=downsample(time,reducedSampleFactor)

plot(timeSub,costJSub)
% ylim([0,5000])
set(gca,'FontSize', 14);
hold on
title('J');xlabel('Time (s)'); ylabel('J')


% %% plot ISE space
% clc;
% 
% K=1;
% tau=1;
% % L=10;
% 
% if L<1 %big searching space(Kp,,ki>=100 for L<0.1)
%     kp=1:1:15; ki=1:15/length(kp):15;
%     t=0:0.1:10;
% elseif L>=1 && L<10 %average searching space (kp,ki<1 for L>10)
%     kp=0.1:0.1:1; ki=0.1:1/length(kp):1;%+1/length(kp);
%     t=0:0.1:100;
% elseif L>=10 %small searching space (kp,ki<1 for L>10)
%     kp=0.01:0.01:0.1; ki=0.01:0.01:0.1;
%     t=0:0.1:1500;
% end
% 
% refAmplitude=1;
% R=refAmplitude*ones(1,length(t));

% %%  plot obtained gains vs ZN and ZNM L=0.1
% s=tf('s');
% p=((1*exp(-L*s))/(1*s+1));
% [Gc01,KpZN01,TiZN01,TdZN01,H]=ziegler(2,[1 L 1, 10]);      %original ZN
% [GcM01,KpZNM01,TiZNM01,TdZNM01,HM]=ziegler(3,[1 L 1, 10]); %modified ZN
% KiZN01=KpZN01/TiZN01;
% KiZNM01=KpZNM01/TiZNM01;
% 
% %closed loop response of the system with ZN PI tuning parameters
% T=feedback(GcM01*p,1);
% Y=lsim(T,R,t);
% figure()
% plot(t,Y)
% hold on
% YZN=Y;
% 
% %closed loop response of the system with Modified ZN PI tuning parameters
% T=feedback(Gc01*p,1);
% Y=lsim(T,R,t);
% plot(t,Y)
% 
% %closed loop response of the system with SOC tuning parameters
% GcSOC= kpOptim(end)+kiOptim(end)/s;
% T=feedback(GcSOC*p,1);
% Y=lsim(T,R,t);
% plot(t,Y)
% YSOC=Y;
% 
% legend('PI ZN','PI ZNM','PI SOC')
% xlabel('Time (s)')
% set(gca,'FontSize', 14);


%% SOC performance indices
SOCConvTime= timeSub(  converTimeInv(costJSub,10,Ts)); %convergence time SOC
OVFinal=OVOptim(end);   %Final OV
TSFinal=TSOptim(end);   %Final Settling time
RMSFinal=rms(yOut);             %RMS value of the system output
ISEFinal=     (1/length(yOut))*sum((yOut-ref).^2);
IAEFinal=     (1/length(yOut))*sum(abs(yOut));
ITAEFinal=    (1/length(yOut))*sum(time.*(yOut-ref).^2);
RMSE=    sqrt(  ((1/length(yOut)))*sum(yOut-ref).^2);
varNames={'SOC convergence time','OV','Settling Time','RMS','ISE','IAE','RMSE'};
table(SOCConvTime,OVFinal,TSFinal,RMSFinal,ISEFinal,IAEFinal,RMSE,...
    'VariableNames',varNames)


