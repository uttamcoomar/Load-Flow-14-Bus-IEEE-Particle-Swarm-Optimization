% objective function multiplied by 100 on 7th September , Monday , 2015
clc
nbus =14; % IEEE-5
Y = ybusppg(nbus); % Calling ybusppg.m to get Y-Bus Matrix..
busd = busdatas(nbus);
BMva=100; % Calling busdatas.. % Base MVA..
Pg = busd(:,5)/BMva; % gen erated real power
Qg = busd(:,6)/BMva; % generated reactive power.
Pl = busd(:,7)/BMva; % load real power
Ql = busd(:,8)/BMva; % load reactive power
Qlim1 = busd(:,9)/BMva;
Qlim2 = busd(:,10)/BMva;
P = Pg - Pl; % Pi = PGi - PLi..
Q = Qg - Ql; % Qi = QGi - QLi..
Psp = P ; % P Specified..
Qsp = Q ; % q specified
for i=[2,3,6,8]
Qmin(i)=Qlim1(i)-Ql(i);
Qmax(i)=Qlim2(i)-Ql(i);
end
G = real(Y) ; % Conductance matrix..
B = imag(Y) ; % Susceptance matrix..
%kv=input('kv')
%---------------------PSO PARAMETERS INITIALIZATION --------------%
fev=0;
c12=0;
kcm=1;
kpm=input('enter kpm');
kqm=input('enter kqm');
%kp=input('enter kp');
%kq=input('enter kq');
%kthm=input('enter kthm');
p=input('enter no. of initial particles'); % no of particle
it=input('enter maximum no. of iterations'); % no of iteration
rp=1;
T=input('enter tolerance'); %tolerance factor
particle=[];
mn=[];
fr1=[];
fs=input('frequency for sorting');
pf=input('final no. of particles'); % no of particle
r=input('enter r');
tic
c12=[];
rg=1;
rf=1;
f=zeros(p,it);
MQS1=zeros(4,1);
fp=zeros(p,1);
thp=zeros(p,nbus);
thg=zeros(p,nbus);
vp=zeros(p,nbus);
vg=zeros(p,nbus);
v=zeros(p,it,nbus);
th=zeros(p,it,nbus);
vth=zeros(p,it,nbus);
vv=zeros(p,it,nbus);
a=-0.5;
b=0.5;
vth(:,1,:)=a+(b-a)*rand(p,nbus); %initial velocity of theta vector%
a=.1;
b=-0.1;
vv(:,1,:)=a+(b-a)*rand(p,nbus); %initial velocity of voltage vector%
%vth(:,:,1)=0;
a=0;
b=0;
th(:,1,:)=a+(b-a)*rand(p,nbus);
a=1;
b=1;
v(:,1,:)=a+(b-a)*rand(p,nbus);
% volage asuumption
v(:,:,1)=1.060;
v(:,:,2)=1.045;
v(:,:,3)=1.010;
v(:,:,6)=1.070;
v(:,:,8)=1.090;
vv(:,:,1)=0;
th(:,:,1)=0;
thp(:,1)=0;
thg(:,1)=0;
vp(:,1)=1.060;
vp(:,2)=1.045;
vp(:,3)=1.010;
vp(:,6)=1.070;
vp(:,8)=1.090;
vg(:,1)=1.060;
vg(:,2)=1.045;
vg(:,3)=1.010;
vg(:,6)=1.070;
vg(:,8)=1.090;
vmin=[0.9 0.9 0.9 0.9 0.9 0.9 0.9 0.9 0.9 0.9 0.9 0.9 0.9 0.9];
vmax=[1.06 1.045 1.010 1.1 1.1 1.070 1.1 1.090 1.1 1.1 1.1 1.1 1.1 1.1];
%-----------------------initial value of objective function-------------%
% Calculate P and Q
PVIND=zeros(p,nbus);
MV=zeros(p,nbus);
check_no=0;
for j=1:p
P = zeros(nbus,1);
Q = zeros(nbus,1);
MPS=zeros(14,1);
MQS=zeros(14,1);
MQS1=zeros(4,1);
MVS=zeros(14,1);
for i = 1:nbus
for k = 1:nbus
P(i) = P(i) + v(j,1,i)* v(j,1,k)*(G(i,k)*cos(th(j,1,i)-th(j,1,k)) + B(i,k)*sin(th(j,1,i)-th(j,1,k)));
end
end
for i = 1:nbus
for k = 1:nbus
Q(i) = Q(i) + v(j,1,i)* v(j,1,k)*(G(i,k)*sin(th(j,1,i)-th(j,1,k)) - B(i,k)*cos(th(j,1,i)-th(j,1,k)));
end
end
% real power mismatch
MP=P-Psp;
if (P(1)-.5)*(2-(P(1))) >= 0
MP(1)=0;
end
if (P(1)-.5)*(2-(P(1))) < 0
MP(1)=abs(min((P(1)-.5),(2-(P(1)))));
end
%if (P(2)-.2)*(1-(P(2))) >= 0
% MP(2)=P(2)-Psp(2);
%end
%if (P(2)-.2)*(1-(P(2))) < 0
% MP(2)=abs(min((P(2)-.5),(2-(P(2)))));
%end
MPS=MP.^2;
%reactive power mismatch and voltage mismatch
MQ=Q-Qsp;
MQS=MQ.^2;
MQS(1)=0;
% MVS=zeros(14,1);
% for jk=2:14
% if (v(j,1,jk)-0.9)*(1.1-v(j,1,jk)) < 0
% MVS(jk)=(min((v(j,1,jk)-0.9),(1.1-v(j,1,jk))))^2;
% end
% if (v(j,1,jk)-0.9)*(1.1-v(j,1,jk)) >= 0
% MVS(jk)=0;
% end
% end
for jk=[2,3,6,8]
if((Q(jk)-Qmin(jk))*(Qmax(jk)-Q(jk))<0)
% MVS(jk)=((v(j,1,jk)-vmax(jk))^2);
MQS(jk)=(min((Q(jk)-Qmin(jk)),(Qmax(jk)-Q(jk))))^2;
vv(j,1,jk)=0; %change made at 1906 hrs , 6th September , Sunday , 2015
end
if((Q(jk)-Qmin(jk))*(Qmax(jk)-Q(jk))>=0)
MQS(jk)=0;
% MVS(jk)=(vmax(jk)-v(j,1,jk))^2;
vv(j,1,jk)=0;
end
end
%objective function value
% f(j,1)=max(max(kpm*max(MPS),kqm*max(MQS)),max(kvm*max(MVS),kthm*sum(MTHS)));
f(j,1)=(kpm*sum(MPS)+kqm*sum(MQS));%+kvm*sum(MVS);%+kthm*sum(MTHS);
%fr(j,1)=f(j,1);
%fr(j,2)=j;
fev=fev+1;
end
%Initial personal best values
for i=1:p
for k=1:nbus
thp(i,k)=th(i,1,k);
end
for k=1:nbus
vp(i,k)=v(i,1,k);
end
end
%for Initial Global best values updation
fmin=min(f(:,1));
for k=1:p
if f(k,1)==fmin
gb=k;
else
end
end
%Initial global best value
for k=1:p
for j=1:nbus
thg(k,j)=th(gb,1,j);
end
for j=1:nbus
vg(k,j)=v(gb,1,j);
end
end
fgm = min(f(:,1));
Q3=zeros(p,it,nbus);
for i=1:it
%for inertia weight W
%wmax=.4;
%wmin=.3;
%w=wmax-((wmax-wmin)*i/it);
%velocity update
%position update
w=0.8;
for j=1:p
% w(j)=.4+(f(j,i)*(min(f(:,i)-f(j,i)))/(fp(j)*(min(f(:,i)-f(j,i)))));
% L1(j)=sqrt(fp(j)/f(j,i));
% L2(j)=sqrt(min(f(:,i))/f(j,i));
for k=1:nbus
vth(j,(i+1),k) = w*vth(j,i,k) + rp*rand()*(thp(j,k)-th(j,i,k)) + rg*rand()*(thg(j,k)-th(j,i,k));
th(j,(i+1),k) = th(j,i,k) + rf*vth(j,(i+1),k);
end
for q=2:nbus
vv(j,(i+1),q) = w*vv(j,i,q) + rp*rand()*(vp(j,q)-v(j,i,q)) + rg*rand()*(vg(j,q)-v(j,i,q));
v(j,(i+1),q) = v(j,i,q) + rf*vv(j,(i+1),q);
end
end
%objective function value
for j=1:p
P = zeros(nbus,1);
Q = zeros(nbus,1);
MPS=zeros(14,1);
MQS=zeros(14,1);
MQS1=zeros(14,1);
for m = 1:nbus
for k = 1:nbus
P(m) = P(m) + v(j,(i+1),m)* v(j,(i+1),k)*(G(m,k)*cos(th(j,(i+1),m)-th(j,(i+1),k)) + B(m,k)*sin(th(j,(i+1),m)-th(j,(i+1),k)));
end
end
for m = 1:nbus
for k = 1:nbus
Q(m) = Q(m) + v(j,(i+1),m)* v(j,(i+1),k)*(G(m,k)*sin(th(j,(i+1),m)-th(j,(i+1),k)) - B(m,k)*cos(th(j,(i+1),m)-th(j,(i+1),k)));
end
end
% real power mismatch
MP=P-Psp;
if (P(1)-.5)*(2-(P(1))) >= 0
MP(1)=0;
end
if (P(1)-.5)*(2-(P(1))) < 0
MP(1)=abs(min((P(1)-.5),(2-(P(1)))));
end
% if (P(2)-.2)*(1-(P(2))) >= 0
% MP(2)=P(2)-Psp(2);
% end
% if (P(2)-.2)*(1-(P(2))) < 0
% MP(2)=abs(min((P(2)-.5),(2-(P(2)))));
% end
MPS=MP.^2;
%reactive power mismatch and voltage mismatch
MQ=Q-Qsp;
MQS=MQ.^2;
MQS(1)=0;
% MVS=zeros(14,1);
% for jk=2:14
% if (v(j,1,jk)-0.9)*(1.1-v(j,1,jk)) < 0
% MVS(jk)=(min((v(j,1,jk)-0.9),(1.1-v(j,1,jk))))^2;
% end
% if (v(j,1,jk)-0.9)*(1.1-v(j,1,jk)) >= 0
% MVS(jk)=0;
% end
% end
for jk=[2,3,6,8]
if((Q(jk)-Qmin(jk))*(Qmax(jk)-Q(jk))<0)
% MVS(jk)=((v(j,1,jk)-vmax(jk))^2);
MQS(jk)=(min((Q(jk)-Qmin(jk)),(Qmax(jk)-Q(jk))))^2;
end
if((Q(jk)-Qmin(jk))*(Qmax(jk)-Q(jk))>=0)
MQS(jk)=0;
end
end
%objective function value
%f(j,i+1)=max(max(kpm*max(MPS),kqm*max(MQS)),max(kvm*max(MVS),kthm*sum(MTHS)));
f(j,(i+1))=(kpm*sum(MPS)+kqm*sum(MQS));%+kvm*sum(MVS);%+kthm*sum(MTHS);
fr(j,1)=f(j,i+1);
fr(j,2)=j;
fev=fev+1;
end
%personal best values updation
for j=1:p
P = zeros(nbus,1);
Q = zeros(nbus,1);
MPS=zeros(p,1);
MQS=zeros(p,1);
for t =1:nbus
for k = 1:nbus
P(t) = P(t) + vp(j,t)* vp(j,k)*(G(t,k)*cos(thp(j,t)-thp(j,k)) + B(t,k)*sin(thp(j,t)-thp(j,k)));
end
end
for t = 1:nbus
for k = 1:nbus
Q(t) = Q(t) + vp(j,t)* vp(j,k)*(G(t,k)*sin(thp(j,t)-thp(j,k)) - B(t,k)*cos(thp(j,t)-thp(j,k)));
end
end
% real power mismatch
MP=P-Psp;
if (P(1)-.5)*(2-(P(1))) >= 0
MP(1)=0;
end
if (P(1)-.5)*(2-(P(1))) < 0
MP(1)=abs(min((P(1)-.5),(2-(P(1)))));
end
% if (P(2)-.2)*(1-(P(2))) >= 0
% MP(2)=P(2)-Psp(2);
% end
% if (P(2)-.2)*(1-(P(2))) < 0
% MP(2)=abs(min((P(2)-.5),(2-(P(2)))));
% end
MPS=MP.^2;
%reactive power mismatch and voltage mismatch
MQ=Q-Qsp;
MQS=MQ.^2;
MQS(1)=0;
for jk=[2,3,6,8]
if((Q(jk)-Qmin(jk))*(Qmax(jk)-Q(jk))<0)
% MVS(jk)=((v(j,1,jk)-vmax(jk))^2);
MQS(jk)=(min((Q(jk)-Qmin(jk)),(Qmax(jk)-Q(jk))))^2;
end
if((Q(jk)-Qmin(jk))*(Qmax(jk)-Q(jk))>=0)
MQS(jk)=0;
end
end
%objective function value
%fp(j)=max(max(kpm*max(MPS),kqm*max(MQS)),max(kvm*max(MVS),kthm*sum(MTHS)));
fp(j)=(kpm*sum(MPS)+kqm*sum(MQS));%+kvm*sum(MVS);%+kthm*sum(MTHS);
fev=fev+1;
end
if(p>pf && mod(i,fs)==0)
fr=sortrows(fr);
fr1=fr;
k1=1;
k2=0;
k3=((p/r));
k4=1;
for i1=(k3+1):p
mn(1,k1)=fr(i1,2);
k1=k1+1;
end
v(mn,:,:)=[];
th(mn,:,:)=[];
vv(mn,:,:)=[];
vth(mn,:,:)=[];
vp(mn,:)=[];
thp(mn,:)=[];
vg(mn,:)=[];
thg(mn,:)=[];
f(mn,:,:)=[];
fr(mn,:)=[];
p=p/r;
end
mn=[];
%personal best value updation
for k=1:p
for m=1:nbus
if f(k,i+1)<fp(k)
thp(k,m)=th(k,i+1,m);
vp(k,m)=v(k,i+1,m);
else
end
end
end
%for Global best values updation
% if min(f(:,(i+1)))<fgm
fgm=min(f(:,(i+1)));
%end
%end
for m=1:nbus
for k=1:p
if f(k,i+1)==fgm
for l=1:p
thg(l,m) = th(k,i+1,m); %global best values
end
else
end
end
end
for m=2:nbus
for k=1:p
if f(k,i+1)==fgm
for l=1:p
vg(l,m) = v(k,i+1,m); %global best values
end
else
end
end
end
%stopping criteria
% gb1=gb;
fgm=min(f(:,(i)));
for k=1:p
if f(k,i)==fgm
gb1=k;
else
end
end
if (abs(f(gb1,i))<=10^(-T))
break
end
%vth(gb1,(i+1),2)
end
toc;
c12=toc;
P=zeros(14,1);
Q=zeros(14,1);
for m = 1:nbus
for k = 1:nbus
P(m) = P(m) + v(gb1,(i),m)* v(gb1,(i),k)*(G(m,k)*cos(th(gb1,(i),m)-th(gb1,(i),k)) + B(m,k)*sin(th(gb1,(i),m)-th(gb1,(i),k)));
end
end
for m = 1:nbus
for k = 1:nbus
Q(m) = Q(m) + v(gb1,(i),m)* v(gb1,(i),k)*(G(m,k)*sin(th(gb1,(i),m)-th(gb1,(i),k)) - B(m,k)*cos(th(gb1,(i),m)-th(gb1,(i),k)));
end
end
for j=1:1:14
fprintf('\n power calculated at bus no. %d is %f + j %f',j,P(j),Q(j));
fprintf('\n power demand at bus no. %d is %f + j %f',j,Psp(j),Qsp(j));
fprintf('\n voltage %f',v(gb1,i,j));
fprintf('\n delta %f',th(gb1,i,j)*180/(pi));
fprintf('\n');
end
disp('function value =');
f(gb1,i)
disp('no. of function evaluations');
fev
disp('time elapsed')
c12
disp('no. of iterations')
i
