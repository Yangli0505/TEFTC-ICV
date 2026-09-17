clear; close all; clc;

%%  setting
ts=0.01;
Tend=70;
T=Tend/ts;
num=5;

hat=cell(1,num);
f=cell(1,num);
varepsilon=cell(1,num);
Gamma=cell(1,num);
e=cell(1,num);
s=cell(1,num);
Z=cell(1,num);
D=cell(1,num);
beta=cell(1,num);
ed=cell(1,num);
u=cell(1,num);
u_bar=cell(1,num);
A=zeros(1,T);
B=zeros(1,T);
C=zeros(1,T);
t=zeros(1,T);
r=zeros(num,T);
rho=zeros(num,T);
zeta1=zeros(1,T);
zeta2=zeros(1,T);
zeta3=zeros(1,T);

%RBFNN
neurons=13;

%nonlinear parameters
nu=1.2;
Air=2.2;
Cd=0.35;
dm=5;

%spacing parameters
Delta=7;
h=0.32;

%design parameters
pi=1;
lambda=0.85;

Kp=10;
Ki=10;
Kd=0.3;

eta=0.1;
vartheta=0.1;

d1=0.0001;
b=2;
pi1=0.1;
pi2=0.01;
pi3=0.01;

%hat_(Phi,Xi,Theta)
for iter = 1 : T
    t(iter)= iter*ts;
    zeta1(:,iter)=0.1*exp( -1*t(iter) );
    zeta2(:,iter)=1*exp( -1*t(iter) );
    zeta3(:,iter)=3*exp( -1*t(iter) );
end

for i=1:num
    hat{i}(1,1)=1;
    hat{i}(2,1)=3; 
    hat{i}(3,1)=3; 
end

%%  initialize the states
%leader
p(:,1) = 75; % position
dotp(:,1) = 10; % velocity
ddotp(:,1) = 0; % acceleration

%followers
x{1}(:,1) = [59;   10.5;  0];
x{2}(:,1) = [43.5; 10;  0];
x{3}(:,1) = [28.5; 9.5;  0];
x{4}(:,1) = [14;   9;  0];
x{5}(:,1) = [0;    8.5;  0];

%veh_(m,tau,L)
veh{1}(:,1) = [1500;  0.55;  4  ];
veh{2}(:,1) = [1600;  0.35;  4.5];
veh{3}(:,1) = [1550;  0.44;  5  ];
veh{4}(:,1) = [1650;  0.38;  5  ];
veh{5}(:,1) = [1500;  0.5;   4.5];

%nonlinear function
for i=1:num
    f{i}(:,1)=-( 1/veh{i}(2,1) )*( x{i}(3,1) + ( nu*Air*Cd )/( 2*veh{i}(1,1) ) * ( x{i}(2,1) )^2 + dm/veh{i}(1,1) )...
        - ( nu*Air*Cd*x{i}(2,1)*x{i}(3,1) )/veh{i}(1,1);
    x{i}(4,1)=f{i}(:,1);
end

%varepsilon_(varepsilon, dotvarepsilon, ddvarepsilon)
varepsilon{1}(1,1)=p(:,1)-x{1}(1,1)-veh{1}(3,1)-Delta-h*x{1}(2,1);
varepsilon{1}(2,1)=dotp(:,1)-x{1}(2,1)-h*x{1}(3,1);
varepsilon{1}(3,1)=ddotp(:,1)-x{1}(3,1)-h*x{1}(4,1);

for i=2:num
    varepsilon{i}(1,1)=x{i-1}(1,1)-x{i}(1,1)-veh{i}(3,1)-Delta-h*x{i}(2,1);
    varepsilon{i}(2,1)=x{i-1}(2,1)-x{i}(2,1)-h*x{i}(3,1);
    varepsilon{i}(3,1)=x{i-1}(3,1)-x{i}(3,1)-h*x{i}(4,1);
end

for i=1:num
    A(i)=varepsilon{i}(1,1);
    B(i)=pi*varepsilon{i}(1,1)+varepsilon{i}(2,1);
    C(i)=( pi^2*varepsilon{i}(1,1) + 2*pi*varepsilon{i}(2,1) + varepsilon{i}(3,1) ) / 2;
end

%Gamma_(Gamma,dotGamma,ddotGamma)
for i=1:num
    Gamma{i}(1,1)=A(i);
    Gamma{i}(2,1)=-1*pi*A(i)+B(i);
    Gamma{i}(3,1)=-1*pi*B(i)+2*C(i)-pi*( -pi*A(i)+B(i) );
end

%e_(e,dote,ddote)
for i=1:num
    e{i}(1,1)= varepsilon{i}(1,1)-Gamma{i}(1,1);
    e{i}(2,1)= varepsilon{i}(2,1)-Gamma{i}(2,1);
    e{i}(3,1)= varepsilon{i}(3,1)-Gamma{i}(3,1);
end

%s_(s,S)
for i=1:num
    s{i}(1,1)= Kp*e{i}(1,1) + Ki*ts*e{i}(1,1) + Kd*e{i}(2,1);
end

for i=1:(num-1)
    s{i}(2,1)= lambda*s{i}(1,1) - s{i+1}(1,1);
end
s{num}(2,1)= lambda*s{num}(1,1);

%Z
Z{1}(:,1)= lambda*Kp*e{1}(2,1) + lambda*Ki*e{1}(1,1) + lambda*Kd*( ddotp(:,1)-x{1}(3,1)-Gamma{1}(3,1) )...
    -Kp*e{2}(2,1) - Ki*e{2}(1,1) - Kd*e{2}(3,1);
for i=2:(num-1)
    Z{i}(:,1)= lambda*Kp*e{i}(2,1) + lambda*Ki*e{i}(1,1) + lambda*Kd*( x{i-1}(3,1)-x{i}(3,1)-Gamma{i}(3,1) )...
        -Kp*e{i+1}(2,1) - Ki*e{i+1}(1,1) - Kd*e{i+1}(3,1);
end
Z{num}(:,1)= lambda*Kp*e{num}(2,1) + lambda*Ki*e{num}(1,1) + lambda*Kd*( x{num-1}(3,1)-x{num}(3,1)-Gamma{num}(3,1) );

RBFNN_phi= [linspace(-2,5,(neurons));linspace(3.5,4,(neurons))];
RBFNN_theta = 1.2;

for i=1:num
    for j = 1:neurons
        D{i}(j) = exp( - ( ((x{i}(2:3,1) - RBFNN_phi(:,j))'*(x{i}(2:3,1) - RBFNN_phi(:,j))) / (2 * RBFNN_theta^2))   );
    end
    D{i}=D{i}/sum(D{i});
end

%β
for i=1:num
    beta{i}(:,1)=(1/b^2)*0.5*hat{i}(1,1)*D{i}*D{i}'*s{i}(2,1)+tanh(s{i}(2,1)/d1)*hat{i}(2,1)+(1/(lambda*h*Kd))*s{i}(2,1);
end

%%  Simulation
for iter = 1 : T
    if iter <1/ts
        ddotp(:,iter+1)=0;
    elseif iter <4/ts
        ddotp(:,iter+1)=2/3*t(iter)-2/3;
    elseif iter <8/ts
        ddotp(:,iter+1)=2;
    elseif iter <12/ts
        ddotp(:,iter+1)=-0.5*t(iter)+6;
    elseif iter <30/ts
        ddotp(:,iter+1)=0;
    elseif iter <35/ts
        ddotp(:,iter+1)=-0.2*t(iter)+6;
    elseif iter <40/ts
        ddotp(:,iter+1)=-1;
    elseif iter <45/ts
        ddotp(:,iter+1)=0.2*t(iter)-9;
    else
        ddotp(:,iter+1)=0;
    end

    for i=1:num
        %actuator fault "r" for bias and "rho" for partial loss of actuation effectiveness
        if iter <10/ts
            rho(i,iter)=1;
        elseif iter <40/ts
            r(2,iter)=0.1*sin(0.5*t(iter));
            r(4,iter)=0.1*sin(0.5*t(iter));
            rho(1,iter)=1;
            rho(2,iter)=0.6;
            rho(3,iter)=1;
            rho(4,iter)=0.49;
            rho(5,iter)=1;
        elseif iter <70/ts
            r(2,iter)=0.15*sin(0.5*t(iter));
            r(4,iter)=0.15*sin(0.5*t(iter));
            rho(1,iter)=1;
            rho(2,iter)=0.55;
            rho(3,iter)=1;
            rho(4,iter)=0.45;
            rho(5,iter)=1;
        end

        %external disturbance
        ed{i}(:,iter)=0.01*sin(t(iter));    

        u{i}(:,iter)=s{i}(2,iter)*hat{i}(3,iter)^2*beta{i}(1,iter)^2/sqrt( s{i}(2,iter)^2*hat{i}(3,iter)^2*beta{i}(1,iter)^2+eta )...
            +(1/(lambda*h*Kd))*s{i}(2,iter)*hat{i}(3,iter)^2*Z{i}(1,iter)^2/sqrt(s{i}(2,iter)^2*hat{i}(3,iter)^2*Z{i}(1,iter)^2+vartheta);

        u_bar{i}(:,iter)= rho(i,iter)*sat( u{i}(:,iter) )+r(i,iter);

        %update state
        dotp(:,iter+1) = ts*ddotp(:,iter+1)+dotp(:,iter);
        p(:,iter+1) = ts*dotp(:,iter+1)+p(:,iter);
        x{i}(4,iter+1)=f{i}(:,iter)+(1/veh{i}(2,1))*u_bar{i}(:,iter)+ed{i}(:,iter);
        x{i}(3,iter+1)=sat( x{i}(4,iter+1)*ts+x{i}(3,iter) );
        x{i}(2,iter+1)=x{i}(3,iter+1)*ts+x{i}(2,iter);
        x{i}(1,iter+1)=x{i}(2,iter+1)*ts+x{i}(1,iter);

        f{i}(:,iter+1)=-( 1/veh{i}(2,1) )*( x{i}(3,iter+1) + ( nu*Air*Cd )/( 2*veh{i}(1,1) ) * ( x{i}(2,iter+1) )^2 + dm/veh{i}(1,1) )...
            - ( nu*Air*Cd*x{i}(2,iter+1)*x{i}(3,iter+1) )/veh{i}(1,1);
        for j = 1:neurons
            D{i}(j) = exp( - ( ((x{i}(2:3,iter+1) - RBFNN_phi(:,j))'*(x{i}(2:3,iter+1) - RBFNN_phi(:,j))) / (2 * RBFNN_theta^2)));
        end
        D{i}=D{i}/sum(D{i});
    end

    %varepsilon_(varepsilon, dotvarepsilon, ddvarepsilon)
    varepsilon{1}(1,iter+1)=p(:,iter+1)-x{1}(1,iter+1)-veh{1}(3,1)-Delta-h*x{1}(2,iter+1);
    varepsilon{1}(2,iter+1)=dotp(:,iter+1)-x{1}(2,iter+1)-h*x{1}(3,iter+1);
    varepsilon{1}(3,iter+1)=ddotp(:,iter+1)-x{1}(3,iter+1)-h*x{1}(4,iter+1);

    for i=2:num
        varepsilon{i}(1,iter+1)=x{i-1}(1,iter+1)-x{i}(1,iter+1)-veh{i}(3,1)-Delta-h*x{i}(2,iter+1);
        varepsilon{i}(2,iter+1)=x{i-1}(2,iter+1)-x{i}(2,iter+1)-h*x{i}(3,iter+1);
        varepsilon{i}(3,iter+1)=x{i-1}(3,iter+1)-x{i}(3,iter+1)-h*x{i}(4,iter+1);
    end

    %Gamma_(Gamma,dotGamma,ddotGamma)
    for i=1:num
        Gamma{i}(1,iter+1)=( A(i) + B(i) *t(iter) + C(i) *t(iter)*t(iter) )*exp(-pi*t(iter));
        Gamma{i}(2,iter+1)=( -pi*( A(i) + B(i) *t(iter) + C(i) *t(iter)*t(iter)) + (B(i) + 2*C(i)*t(iter)) )*exp(-pi*t(iter));
        Gamma{i}(3,iter+1)=( -pi*( B(i)+2*C(i)*t(iter) )+2*C(i) )*exp(-pi*t(iter))-pi*( -pi*( A(i) + B(i) *t(iter) + C(i) *t(iter)*t(iter) ) + (B(i) + 2*C(i)*t(iter)) )*exp(-pi*t(iter));
        e{i}(1,iter+1)= varepsilon{i}(1,iter+1)-Gamma{i}(1,iter+1);
        e{i}(2,iter+1)= varepsilon{i}(2,iter+1)-Gamma{i}(2,iter+1);
        e{i}(3,iter+1)= varepsilon{i}(3,iter+1)-Gamma{i}(3,iter+1);
    end

    %s_(s,S)
    for i=1:num
        s{i}(1,iter+1)= Kp*e{i}(1,iter+1) + Ki*ts*sum(e{i}(1,1:(iter+1))) + Kd*e{i}(2,iter+1);
    end

    for i=1:(num-1)
        s{i}(2,iter+1)= lambda*s{i}(1,iter+1) - s{i+1}(1,iter+1);
    end
    s{num}(2,iter+1)= lambda*s{num}(1,iter+1);

    %Z
    Z{1}(:,iter+1)= lambda*Kp*e{1}(2,iter+1) + lambda*Ki*e{1}(1,iter+1) + lambda*Kd*( ddotp(:,iter+1)-x{1}(3,iter+1)-Gamma{1}(3,iter+1) )...
        -Kp*e{2}(2,iter+1) - Ki*e{2}(1,iter+1) - Kd*e{2}(3,iter+1);
    for i=2:(num-1)
        Z{i}(:,iter+1)= lambda*Kp*e{i}(2,iter+1) + lambda*Ki*e{i}(1,iter+1) + lambda*Kd*( x{i-1}(3,iter+1)-x{i}(3,iter+1)-Gamma{i}(3,iter+1) )...
            -Kp*e{i+1}(2,iter+1) - Ki*e{i+1}(1,iter+1) - Kd*e{i+1}(3,iter+1);
    end
    Z{num}(:,iter+1)= lambda*Kp*e{num}(2,iter+1) + lambda*Ki*e{num}(1,iter+1) + lambda*Kd*( x{(num-1)}(3,iter+1)-x{num}(3,iter+1)-Gamma{num}(3,iter+1) );

    for i=1:num
        hat{i}(1,iter+1)=ts*( pi1*( lambda*h*Kd*0.5*(1/b^2)*D{i}*D{i}'*s{i}(2,iter+1)^2 - zeta1(:,iter)* hat{i}(1,iter)  ) )+ hat{i}(1,iter);
        hat{i}(2,iter+1)=ts*( pi2*( lambda*h*Kd*s{i}(2,iter+1)*tanh(s{i}(2,iter+1)/d1) - zeta2(:,iter)*hat{i}(2,iter) ) )+hat{i}(2,iter);
        beta{i}(:,iter+1)=(1/b^2)*0.5*hat{i}(1,iter+1)*D{i}*D{i}'*s{i}(2,iter+1)+tanh(s{i}(2,iter+1)/d1)*hat{i}(2,iter+1)+(1/(lambda*h*Kd))*s{i}(2,iter+1);
        hat{i}(3,iter+1)=ts*( pi3*( lambda*h*Kd*s{i}(2,iter+1)*beta{i}(:,iter+1) + s{i}(2,iter+1)*Z{i}(:,iter+1) - zeta3(:,iter)*hat{i}(3,iter) ) )+hat{i}(3,iter);
    end
end

%%  figure
lineStyles = {'-', '--', '-', '--', '-', '--'};

colors = {
    [104 190 217]/255,...
    [191 223 210]/255,...
    [239 206 135]/255,...
    [234 165 88]/255,...
    [237 141 90]/255,...
    [37 125 139]/255};

figure(1) % velocity
for i = 1:num
    plot(t(1,1:T),x{i}(2,1:T), 'linewidth',2,'Color', colors{i},'LineStyle', lineStyles{i});
    hold on;
end
plot(t(1,1:T),dotp(1,1:T), 'linewidth',2,'Color', colors{num+1},'LineStyle', lineStyles{num+1});
ylim([8,26]);
yticks(10:5:25);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Veh1', 'Veh2', 'Veh3', 'Veh4', 'Veh5', 'Veh0');

figure(2)  % acceleration
for i = 1:num
    plot(t(1,1:T),x{i}(3,1:T), 'linewidth',2,'Color', colors{i},'LineStyle', lineStyles{i});
    hold on;
end
plot(t(1,1:T),ddotp(1,1:T),  'linewidth',2,'Color', colors{num+1},'LineStyle', lineStyles{num+1});
ylim([-3,3]);
yticks(-3:1:3);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Veh1', 'Veh2', 'Veh3', 'Veh4', 'Veh5', 'Veh0');

figure(3) % spacing
for i = 1:num
    plot(t(1,1:T),e{i}(1,1:T), 'linewidth',2,'Color', colors{i},'LineStyle', lineStyles{i});
    hold on;
end
ylim([-0.02,0.02]);
yticks(-0.02:0.01:0.02);
xlabel('Time (s)');
ylabel('Spacing error (m)');
legend('Veh1', 'Veh2', 'Veh3', 'Veh4', 'Veh5');


function y = sat(x)
threshold1 = 3;
threshold2 = -3;

if x >= threshold1
    y = threshold1;
elseif  x <= threshold2
    y = threshold2;
else
    y = x;
end

end
