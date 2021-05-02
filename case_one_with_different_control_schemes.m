clc;
clear all ;
close all ;
%% the comparison of method in this paper and these in reference [39] on a two joint robotic manipulator systems
%% the simulation step and time
step=0.001;
n=5000;
t=0:step:step*n;
%% robot manipulator model parameters
m1=2; m2=0.85; l1=0.35; l2=0.31; ge=9.8; I1=0.061125; I2=0.02042;

%% control design parameters in this paper
F11_2=0.3; F12_2=0.3; F21_2=3; F22_2=3;  % the parameters of constraint boundary
 
 
T_2=2; epsilon_2=0.02; b1_2=epsilon_2/(sqrt(2)*F11_2); b2_2=epsilon_2/(sqrt(2)*F12_2); % the parameters of performance
k1_2=4; r1_2=2; w1_2=0.001;
k2_2=15; r2_2=10; r3_2=2; r4_2=25;
NNb=3; range =5; hidden2_2 = 1000; Z_2_scale =8; %the parameters of NN
rng('default');% To reproduce the result as reported in the manuscript. You can comment this line to test rand sample performance.
NNa = (2*rand(Z_2_scale,hidden2_2)-1)*range; % to select the nodes randomly for NN within the scope [-range,range] 

%% initial value in this paper
q1_2=0.15; q2_2=0.4; dq1_2=0; dq2_2=0;ddq1_2=0;ddq2_2=0;
alpha2f_2=0; hata_2=0;


%% control design parameters in  [39]
k0_1=1;    k111_1=60;  k211_1= 14;   fn1_1=12;      alpha_1=0.85;   k122_1=300;  k222_1=4;    fn2_1=10;     
gamma_1=0.005*eye(13); 
eta_1=[0.01;  0.01];      
beta_1=0.5; mu_1=0.9;

%% initial value in  [39]
q1_1=0.15; q2_1=0.4; dq1_1=0; dq2_1=0;ddq1_1=0;ddq2_1=0;
hatphi_1=[0  0  0  0  0 0  0  0  0  0  0  0  0];

for i=1:1:n
    tt=i*step;
    %% desired signals and its derivative
    qd1=0.3*sin(tt); dqd1=0.3*cos(tt); ddqd1=-0.3*sin(tt);
    qd2=0.3*cos(tt); dqd2=-0.3*sin(tt); ddqd2=-0.3*cos(tt);
    qd=[qd1; qd2];dqd=[dqd1;dqd2]; ddqd=[ddqd1;ddqd2];
    
    %% settling time regulator in this paper
    if tt<T_2
        kappa_2=((T_2-tt)/T_2)^3; dkappa_2=-3/T_2 * ((T_2-tt)/T_2)^2;ddkappa_2=6/T_2^2 * ((T_2-tt)/T_2); %%% kappa in (12)
    else if tt>=T_2
            kappa_2=0; dkappa_2=0; ddkappa_2=0;
        end
    end
    
    
    beta1_2=(1-b1_2)*kappa_2+b1_2; dbeta1_2=(1-b1_2)*dkappa_2;
    beta2_2=(1-b2_2)*kappa_2+b2_2; dbeta2_2=(1-b2_2)*dkappa_2;  %%% beta in (13)
    %% system transformation
    e11_2=q1_2-qd1; de11_2=dq1_2-dqd1;
    e12_2=q2_2-qd2; de12_2=dq2_2-dqd2;
    hate11_2=e11_2/beta1_2; hate12_2=e12_2/beta2_2;
    
    if hate11_2<=-F11_2
        h11_2=0; dh11_2=0;
    else if hate11_2>-F11_2 && hate11_2<0
            h11_2= 6*(hate11_2/F11_2)^5 + 15*(hate11_2/F11_2)^4+10*(hate11_2/F11_2)^3+1; %%% e_11 (18)
            dh11_2=30*(hate11_2/F11_2)^4 + 60*(hate11_2/F11_2)^3+30*(hate11_2/F11_2)^2;
        else if  hate11_2>=0   && hate11_2<F11_2
                h11_2= -6*(hate11_2/F11_2)^5 + 15*(hate11_2/F11_2)^4-10*(hate11_2/F11_2)^3+1;
                dh11_2=-30*(hate11_2/F11_2)^4 + 60*(hate11_2/F11_2)^3-30*(hate11_2/F11_2)^2;
            else if  hate11_2>=F11_2
                    h11_2= 0;  dh11_2=0;
                end
            end
        end
    end
    
    if hate12_2<=-F12_2
        h12_2=0; dh12_2=0;
    else if hate12_2>-F12_2 && hate12_2<0
            h12_2= 6*(hate12_2/F12_2)^5 + 15*(hate12_2/F12_2)^4+10*(hate12_2/F12_2)^3+1; %%% e_12 (18)
            dh12_2=30*(hate12_2/F12_2)^4 + 60*(hate12_2/F12_2)^3+30*(hate12_2/F12_2)^2;
        else if  hate12_2>=0   && hate12_2<F12_2
                h12_2= -6*(hate12_2/F12_2)^5 + 15*(hate12_2/F12_2)^4-10*(hate12_2/F12_2)^3+1;
                dh12_2=-30*(hate12_2/F12_2)^4 + 60*(hate12_2/F12_2)^3-30*(hate12_2/F12_2)^2;
            else if  hate12_2>=F12_2
                    h12_2= 0;  dh12_2=0;
                end
            end
        end
    end
    
    x21_2=dq1_2; x22_2=dq2_2;
    
    
    if x21_2<=-F21_2
        h21_2=0; dh21_2=0;
    else if x21_2<0 && x21_2>-F21_2
            h21_2= -2*(x21_2/F21_2)^3-3*(x21_2/F21_2)^2+1; %%% x_21 in (19)
            dh21_2=-6*(x21_2/F21_2)^2 -6*(x21_2/F21_2);
        else if  x21_2>=0 && x21_2<F21_2
                h21_2= 2*(x21_2/F21_2)^3-3*(x21_2/F21_2)^2+1;
                dh21_2=6*(x21_2/F21_2)^2 -6*(x21_2/F21_2);
            else if  x21_2>=F21_2
                    h21_2= 0;  dh21_2=0;
                end
            end
        end
    end
    
    if x22_2<=-F22_2
        h22_2=0; dh22_2=0;
    else if x22_2<0 && x22_2>-F22_2
            h22_2= -2*(x22_2/F22_2)^3-3*(x22_2/F22_2)^2+1; %%% x_21 in (19)
            dh22_2=-6*(x22_2/F22_2)^2 -6*(x22_2/F22_2);
        else if  x22_2>=0 && x22_2<F22_2
                h22_2= 2*(x22_2/F22_2)^3-3*(x22_2/F22_2)^2+1;
                dh22_2=6*(x22_2/F22_2)^2 -6*(x22_2/F22_2);
            else if  x22_2>=F22_2
                    h22_2= 0;  dh22_2=0;
                end
            end
        end
    end
    
    
    xi11_2=e11_2/h11_2;  xi12_2=e12_2/h12_2; %%% new variables in (20)
    xi21_2=x21_2/h21_2;  xi22_2=x22_2/h22_2;
    
    
    
    
    mu11_2=1/h11_2- (dh11_2/F11_2)*e11_2/(beta1_2*h11_2^2);
    mu12_2=1/h12_2- (dh12_2/F12_2)*e12_2/(beta2_2*h12_2^2);
    
    v11_2=(dh11_2/F11_2)*(dbeta1_2*e11_2^2)/(beta1_2^2*h11_2^2);
    v12_2=(dh12_2/F12_2)*(dbeta2_2*e12_2^2)/(beta2_2^2*h12_2^2);
    
    mu21_2=1/h21_2-dh21_2*x21_2/h21_2^2;
    mu22_2=1/h22_2-dh22_2*x22_2/h22_2^2;            %%% (22)
    
    mu1_2=[mu11_2  0; 0    mu12_2];
    R2_2=[h21_2  0; 0  h22_2];
    v1_2=[v11_2;v12_2];
    mu2_2=[mu21_2 0; 0 mu22_2];
    
    %%%%%%%%%%  control design in this paper 
    phi_2=norm(mu1_2)^2*(norm(R2_2)^2+norm(qd)^2)+norm(v1_2)^2;
    z1_2=[xi11_2; xi12_2];
    alpha1_2=-(k1_2+r1_2*phi_2)*z1_2;     %%%%%%%%% (35)
    
   dalpha2f_2=1/w1_2 *(inv(mu1_2*R2_2)*alpha1_2-alpha2f_2);
    alpha2f_2=alpha2f_2+dalpha2f_2*step;
    
    
    xi2_2=[xi21_2; xi22_2];
    z2_2=xi2_2-alpha2f_2;
    Z_2=[ q1_2; q2_2;qd1;qd2;h11_2;h12_2;h21_2;h22_2];
    
    for j=1:hidden2_2
        Sz(j)=exp(-((norm(Z_2-NNa(:,j)))^2)/NNb^2); %%%  NN
    end
    Phi2_2=norm(Sz);
    
    dhata_2=-r3_2*hata_2+r4_2*Phi2_2*norm(z2_2)^2;
    hata_2=hata_2+dhata_2*step;
    
    u_2=-inv(mu2_2)*(k2_2*z2_2+(mu1_2*R2_2)'*z1_2+r2_2*hata_2*Phi2_2*z2_2);
    
    u1_2=u_2(1,1); u2_2=u_2(2,1);
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% system model 
    M11_2=m1*l1^2+m2*(l1^2+l2^2+2*l1*l2*cos(q2_2))+I1+I2;
    M12_2=m2*(l2^2+l1*l2*cos(q2_2))+I2;
    M22_2=m2*l2^2+I2;
    C11_2=-m2*l1*l2*dq2_2*sin(q2_2);
    C12_2=-m2*l1*l2*(dq1_2+dq2_2)*sin(q2_2);
    C21_2=m2*l1*l2*dq1_2*sin(q2_2);
    C22_2=0;
    G1_2=(m1*l2+m2*l1)*ge*cos(q1_2)+m2*l2*ge*cos(q1_2+q2_2)+0.1*tanh(pi/4);
    G2_2=m2*l2*ge*cos(q1_2+q2_2)+0.1*cos(tt);
    
    M_2=[M11_2  M12_2;  M12_2  M22_2];
    C_2=[C11_2 C12_2; C21_2  C22_2];
    G_2=[G1_2; G2_2];
    
    dq_2=[dq1_2; dq2_2];
    
    ddq_2=inv(M_2)*(-C_2*dq_2-G_2+u_2);
    
    
    ddq1_2=ddq_2(1,1);
    ddq2_2=ddq_2(2,1);
    
    dq1_2=dq1_2+ddq1_2*step;
    dq2_2=dq2_2+ddq2_2*step;
    q1_2=q1_2+dq1_2*step;
    q2_2=q2_2+dq2_2*step;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q1_2s(i)=q1_2; qd1s(i)=qd1;
    q2_2s(i)=q2_2;qd2s(i)=qd2;
     z1_21s(i)=z1_2(1,1); z1_22s(i)=z1_2(2,1); xi11_2s(i)= xi11_2;
    
    upbound1_2=F11_2+qd1; downbound1_2=-F11_2+qd1; upbound2_2=F12_2+qd2; downbound2_2=-F12_2+qd2;
    upbound1s_2(i)=upbound1_2; downbound1s_2(i)=downbound1_2; upbound2s_2(i)=upbound2_2; downbound2s_2(i)=downbound2_2;
    dq1_2s(i)=dq1_2; dq2_2s(i)=dq2_2;
    F21s(i)=F21_2; F22s(i)=F22_2;
    
    upboundpra1_2=sqrt(F11_2^2*beta1_2^2+F12_2^2*beta2_2^2); downboundpra1_2=-sqrt(F11_2^2*beta1_2^2+F12_2^2*beta2_2^2);
    upboundpra1s_2(i)=upboundpra1_2; downboundpra1s_2(i)=downboundpra1_2;
    
    e_2=[e11_2;e12_2];
    es_2(i)=norm(e_2);
    

    u1_2s(i)=u1_2;
    u2_2s(i)=u2_2;
    hata_2s(i)=hata_2;
    

  %% method in reference [34]
  %% auxiliary tracking error s
 q_1=[q1_1; q2_1];dq_1=[dq1_1;dq2_1];% state
 eq_1=qd-q_1; eq1_1=eq_1(1,1); eq2_1=eq_1(2,1);
 deq_1=dqd-dq_1; deq1_1=deq_1(1,1); deq2_1=deq_1(2,1);
 ddeq1_1=ddqd1-ddq1_1; ddeq2_1=ddqd2-ddq2_1;
 jieq1_1=eq1_1+eq1_1*step;  jieq2_1=eq2_1+eq2_1*step;
 s1_1=k0_1*deq1_1+k111_1*eq1_1+k211_1*jieq1_1+fn1_1*abs(eq1_1)^alpha_1*sign(eq1_1);
 ds1_1=k0_1*ddeq1_1+k111_1*deq1_1+k211_1*eq1_1+alpha_1*fn1_1*abs(eq1_1)^(alpha_1-1)*sign(eq1_1)*deq1_1;
 s2_1=k0_1*deq2_1+k122_1*eq2_1+k222_1*jieq2_1+fn2_1*abs(eq2_1)^alpha_1*sign(eq2_1);
 ds2_1=k0_1*ddeq2_1+k122_1*deq2_1+k222_1*eq2_1+alpha_1*fn2_1*abs(eq2_1)^(alpha_1-1)*sign(eq2_1)*deq2_1;
s_1=[s1_1;s2_1];
 %%%%%%%%%% matrix function z
z11_1=ddq1_1+ds1_1; z12_1=(ddq1_1+ds1_1)*cos(q2_1); z13_1=ddq2_1+ds2_1; z14_1=(ddq2_1+ds2_1)*cos(q2_1); z15_1=(dq1_1+s1_1)*sin(q2_1)*dq2_1; 
z16_1=(dq2_1+s2_1)*sin(q2_1)*(dq1_1+dq2_1); z17_1=sin(q1_1); z18_1=sin(q1_1+q2_1); z19_1=0; z110_1=0; z111_1=0;  z112_1=0;  z113_1=0;
z21_1=0; z22_1=0; z23_1=0; z24_1=0; z25_1=0; z26_1=0;  z27_1=0; z28_1=0; z29_1=ddq1_1+ds1_1; z210_1=(ddq1_1+ds1_1)*cos(q2_1);
z211_1=ddq2_1+ds2_1; z212_1=(dq1_1+s1_1)*sin(q2_1)*dq1_1; z213_1=sin(q1_1+q2_1);
 Z_1=0.2.*[z11_1 z12_1 z13_1 z14_1 z15_1 z16_1 z17_1 z18_1 z19_1 z110_1 z111_1 z112_1 z113_1; z21_1 z22_1 z23_1 z24_1 z25_1 z26_1 z27_1 z28_1 z29_1 z210_1 z211_1 z212_1 z213_1];

 %%%%%%%%%%%phi
 phia_1=gamma_1 *(Z_1' * s_1- hatphi_1' * eta_1' *s_1);
dhatphi_1=phia_1;
 hatphi_1=hatphi_1+dhatphi_1'*step; 
 
 psi1_1=20.*[0.7+0.3*(norm(s_1))^(beta_1-1)  0; 0  0.4+0.2*(norm(s_1))^(beta_1-1)];
 psi2_1=15.*[0.7+0.3*(norm(s_1))^(2*beta_1)  0; 0  0.5+0.2*(norm(s_1))^(2*beta_1)];
 
 
 %%%%%%%%%%%%% u
 u_1=Z_1*hatphi_1' + psi1_1*s_1 + psi2_1*s_1*(norm(s_1))^(beta_1-1)/((norm(s_1))^(1+beta_1)+mu_1);
 u11_1=Z_1*hatphi_1'; u22_1=psi1_1*s_1; u33_1= psi2_1*s_1*(norm(s_1))^(beta_1-1)/((norm(s_1))^(1+beta_1)+mu_1);
 
 u111_1=u11_1(1,1);  u112_1=u11_1(2,1);  u221_1=u22_1(1,1);  u222_1=u22_1(2,1);  u331_1=u33_1(1,1);  u332_1=u33_1(2,1); 
 u1_1=u_1(1,1); u2_1=u_1(2,1); 
 
 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% system model
    M11_1=m1*l1^2+m2*(l1^2+l2^2+2*l1*l2*cos(q2_1))+I1+I2;
    M12_1=m2*(l2^2+l1*l2*cos(q2_1))+I2;
    M22_1=m2*l2^2+I2;
    C11_1=-m2*l1*l2*dq2_1*sin(q2_1);
    C12_1=-m2*l1*l2*(dq1_1+dq2_1)*sin(q2_1);
    C21_1=m2*l1*l2*dq1_1*sin(q2_1);
    C22_1=0;
    G1_1=(m1*l2+m2*l1)*ge*cos(q1_1)+m2*l2*ge*cos(q1_1+q2_1)+0.1*tanh(pi/4);
    G2_1=m2*l2*ge*cos(q1_1+q2_1)+0.1*sin(tt);
    
    M_1=[M11_1  M12_1;  M12_1  M22_1];
    C_1=[C11_1 C12_1; C21_1  C22_1];
    G_1=[G1_1; G2_1];
    
    dq_1=[dq1_1; dq2_1];
    
    ddq_1=inv(M_1)*(-C_1*dq_1-G_1+u_1);
    
    
    ddq1_1=ddq_1(1,1);
    ddq2_1=ddq_1(2,1);
    
    dq1_1=dq1_1+ddq1_1*step;
    dq2_1=dq2_1+ddq2_1*step;
    q1_1=q1_1+dq1_1*step;
    q2_1=q2_1+dq2_1*step;

 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 
 q1_1s(i)=q1_1; qd1s(i)=qd1; dq1s_1(i)=dq1_1;dq2s_1(i)=dq2_1;
 q2_1s(i)=q2_1;qd2s(i)=qd2;

 u1s_1(i)=u1_1; u2s_1(i)=u2_1; u111s_1(i)=u111_1; u221s_1(i)=u221_1; u331s_1(i)=u331_1; 
eq1s_1(i)=eq1_1; eq2s_1(i)=eq2_1; e_1=norm(eq_1); es_1(i)=e_1; s1s_1(i)=s1_1; s2s_1(i)=s2_1; 
hatphis_1(i)=norm(hatphi_1);
 
 
    
end

figure(1);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),es_1,'r',t(1:n),es_2,'b',t(1:n), upboundpra1s_2,'g--',t(1:n), downboundpra1s_2,'g--', 'linewidth',2);
h=legend({'scheme in [39]','scheme in this paper','boundary curve'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$||e||$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);



figure(2);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),q1_1s,'r',t(1:n),q1_2s,'b', t(1:n),upbound1s_2,'g--',t(1:n),downbound1s_2,'g--', 'linewidth',2);
h=legend({'scheme in [39]','scheme in this paper','boundary curve'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$q_{1}(t)(rad)$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);

figure(3);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),q2_1s,'r',t(1:n),q2_2s,'b', t(1:n),upbound2s_2,'g--',t(1:n),downbound2s_2,'g--', 'linewidth',2);
h=legend({'scheme in [39]','scheme in this paper','boundary curve'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$q_{2}(t)(rad)$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);


figure(4);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),dq1s_1,'r',t(1:n),dq1_2s,'b',t(1:n),F21s,'g--',t(1:n),-F21s,'g--',  'linewidth',2.2);
h=legend({'scheme in [39]','scheme in this paper','boundary curve'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$\dot{q}_1(t)(rad/s)$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);



figure(5);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),dq2s_1,'r',t(1:n),dq2_2s,'b',t(1:n),F22s,'g--',t(1:n),-F22s,'g--', 'linewidth',1.9);
h=legend({'scheme in [39]','scheme in this paper','boundary curve'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$\dot{q}_2(t)(rad/s)$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);


figure(6);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),u1s_1,'r',t(1:n),u1_2s,'b','linewidth',2);
h=legend({'scheme in [39]','scheme in this paper','boundary curve'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$u_1$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);

figure(7);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),u2s_1,'r',t(1:n),u2_2s,'b','linewidth',2);
h=legend({'scheme in [39]','scheme in this paper','boundary curve'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$u_2$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);


figure(8);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),hata_2s,'b','linewidth',2);
h=legend({'hata_2'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$hata_2$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);




