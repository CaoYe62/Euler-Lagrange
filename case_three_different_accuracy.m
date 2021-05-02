clc;
clear all ;
close all ;

%% the simulation step and time
step=0.001;
n=3000;
t=0:step:step*n;
%% robot manipulator model parameters
m1=2; m2=0.85; l1=0.35; l2=0.31; ge=9.8; I1=0.61125; I2=0.02042;

%% control design parameters with accuracy 0.003

F11_2=0.3; F12_2=0.3; 
 F21_2=3; F22_2=3;
 
 
T_2=1; epsilon_2=0.003; b1_2=epsilon_2/(sqrt(2)*F11_2); b2_2=epsilon_2/(sqrt(2)*F12_2);
k1_2=3; r1_2=1.2; w1_2=0.001;

k2_2=15; r2_2=10; r3_2=2; r4_2=25;
NNb=3; range =5; 
hidden2_2 = 1000;
Z_2_scale =8;


%% control design parameters with accuracy 0.03
F11_1=0.3; F12_1=0.3; 
 F21_1=3; F22_1=3;
 
 
T_1=1; epsilon_1=0.03; b1_1=epsilon_1/(sqrt(2)*F11_1); b2_1=epsilon_1/(sqrt(2)*F12_1);
k1_1=3; r1_1=1.2; w1_1=0.001;
k2_1=15; r2_1=10; r3_1=2; r4_1=25;
hidden2_1 = 1000;
Z_1_scale =8;

%% initial value
q1_2=0.15; q2_2=0.4; dq1_2=0; dq2_2=0;ddq1_2=0;ddq2_2=0;
alpha2f_2=0; hata_2=0;

rng('default');% To reproduce the result as reported in the manuscript. You can comment this line to test rand sample performance.
NNa = (2*rand(Z_2_scale,hidden2_2)-1)*range;

%% initial value
q1_1=0.15; q2_1=0.4; dq1_1=0; dq2_1=0;ddq1_1=0;ddq2_1=0;
alpha2f_1=0; hata_1=0;



for i=1:1:n
    tt=i*step;
    %% desired signals and its derivative
    qd1=0.3*sin(tt); dqd1=0.3*cos(tt); ddqd1=-0.3*sin(tt);
    qd2=0.3*cos(tt); dqd2=-0.3*sin(tt); ddqd2=-0.3*cos(tt);
    qd=[qd1; qd2];dqd=[dqd1;dqd2]; ddqd=[ddqd1;ddqd2];
    
    %% accelerate transformation in (13)
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
    
    
    xi11_2=e11_2/h11_2;  xi12_2=e12_2/h12_2; %%% (20)
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
    
    %%%%%%%%%%  control design
    phi_2=norm(mu1_2)^2*(norm(R2_2)^2+norm(qd)^2)+norm(v1_2)^2;
    z1_2=[xi11_2; xi12_2];
    alpha1_2=-(k1_2+r1_2*phi_2)*z1_2;                %%%%%%%%% (35)
    
    alpha11=-(k1_2)*z1_2;  alpha12=-(r1_2*phi_2)*z1_2;  
   
    alpha11_2s(i)= alpha1_2(1,1); alpha12_2s(i)= alpha1_2(2,1);
     alpha11s(i)=alpha11(1,1); alpha21s(i)=alpha11(2,1);   alpha12s(i)=alpha12(1,1); alpha22s(i)=alpha12(2,1);
    z1_21s(i)=z1_2(1,1); z1_22s(i)=z1_2(2,1); xi11_2s(i)= xi11_2;
    
    dalpha2f_2=1/w1_2 *(inv(mu1_2*R2_2)*alpha1_2-alpha2f_2);
    alpha2f_2=alpha2f_2+dalpha2f_2*step;
    
    
    xi2_2=[xi21_2; xi22_2];
    z2_2=xi2_2-alpha2f_2;
    Z_2=[ q1_2; q2_2;qd1;qd2;h11_2;h12_2;h21_2;h22_2];
    
    for j=1:hidden2_2
        Sz(j)=exp(-((norm(Z_2-NNa(:,j)))^2)/NNb^2);
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
    G1_2=(m1*l2+m2*l1)*ge*cos(q1_2)+m2*l2*ge*cos(q1_2+q2_2);
    G2_2=m2*l2*ge*cos(q1_2+q2_2)+0.1*sin(tt);
    
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
    


   %% accelerate transformation in (13) with accuracy 0.03
    if tt<T_1
        kappa_1=((T_1-tt)/T_1)^3; dkappa_1=-3/T_1 * ((T_1-tt)/T_1)^2;ddkappa_1=6/T_1^2 * ((T_1-tt)/T_1); %%% kappa in (12)
    else if tt>=T_1
            kappa_1=0; dkappa_1=0; ddkappa_1=0;
        end
    end
    
    
    beta1_1=(1-b1_1)*kappa_1+b1_1; dbeta1_1=(1-b1_1)*dkappa_1;
    beta2_1=(1-b2_1)*kappa_1+b2_1; dbeta2_1=(1-b2_1)*dkappa_1;  %%% beta in (13)
    %% system transformation
    e11_1=q1_1-qd1; de11_1=dq1_1-dqd1;
    e12_1=q2_1-qd2; de12_1=dq2_1-dqd2;
    hate11_1=e11_1/beta1_1; hate12_1=e12_1/beta2_1;
    
    if hate11_1<=-F11_1
        h11_1=0; dh11_1=0;
    else if hate11_1>-F11_1 && hate11_1<0
            h11_1= 6*(hate11_1/F11_1)^5 + 15*(hate11_1/F11_1)^4+10*(hate11_1/F11_1)^3+1; %%% e_11 (18)
            dh11_1=30*(hate11_1/F11_1)^4 + 60*(hate11_1/F11_1)^3+30*(hate11_1/F11_1)^2;
        else if  hate11_1>=0   && hate11_1<F11_1
                h11_1= -6*(hate11_1/F11_1)^5 + 15*(hate11_1/F11_1)^4-10*(hate11_1/F11_1)^3+1;
                dh11_1=-30*(hate11_1/F11_1)^4 + 60*(hate11_1/F11_1)^3-30*(hate11_2/F11_1)^2;
            else if  hate11_1>=F11_1
                    h11_1= 0;  dh11_1=0;
                end
            end
        end
    end
    
    if hate12_1<=-F12_1
        h12_1=0; dh12_1=0;
    else if hate12_1>-F12_1 && hate12_1<0
            h12_1= 6*(hate12_1/F12_1)^5 + 15*(hate12_1/F12_1)^4+10*(hate12_1/F12_1)^3+1; %%% e_12 (18)
            dh12_1=30*(hate12_1/F12_1)^4 + 60*(hate12_1/F12_1)^3+30*(hate12_1/F12_1)^2;
        else if  hate12_1>=0   && hate12_1<F12_1
                h12_1= -6*(hate12_1/F12_1)^5 + 15*(hate12_1/F12_1)^4-10*(hate12_1/F12_1)^3+1;
                dh12_1=-30*(hate12_1/F12_1)^4 + 60*(hate12_1/F12_1)^3-30*(hate12_1/F12_1)^2;
            else if  hate12_1>=F12_1
                    h12_1= 0;  dh12_1=0;
                end
            end
        end
    end
    
    x21_1=dq1_1; x22_1=dq2_1;
    
    
    if x21_1<=-F21_1
        h21_1=0; dh21_1=0;
    else if x21_1<0 && x21_1>-F21_1
            h21_1= -2*(x21_1/F21_1)^3-3*(x21_1/F21_1)^2+1; %%% x_21 in (19)
            dh21_1=-6*(x21_1/F21_1)^2 -6*(x21_1/F21_1);
        else if  x21_1>=0 && x21_1<F21_1
                h21_1= 2*(x21_1/F21_1)^3-3*(x21_1/F21_1)^2+1;
                dh21_1=6*(x21_1/F21_1)^2 -6*(x21_1/F21_1);
            else if  x21_1>=F21_1
                    h21_1= 0;  dh21_1=0;
                end
            end
        end
    end
    
    if x22_1<=-F22_1
        h22_1=0; dh22_1=0;
    else if x22_1<0 && x22_1>-F22_1
            h22_1= -2*(x22_1/F22_1)^3-3*(x22_1/F22_1)^2+1; %%% x_21 in (19)
            dh22_1=-6*(x22_1/F22_1)^2 -6*(x22_1/F22_1);
        else if  x22_1>=0 && x22_1<F22_1
                h22_1= 2*(x22_1/F22_1)^3-3*(x22_1/F22_1)^2+1;
                dh22_1=6*(x22_1/F22_1)^2 -6*(x22_1/F22_1);
            else if  x22_1>=F22_1
                    h22_1= 0;  dh22_1=0;
                end
            end
        end
    end
    
    
    xi11_1=e11_1/h11_1;  xi12_1=e12_1/h12_1; %%% (20)
    xi21_1=x21_1/h21_1;  xi22_1=x22_1/h22_1;
    
    
    
    
    mu11_1=1/h11_1- (dh11_1/F11_1)*e11_1/(beta1_1*h11_1^2);
    mu12_1=1/h12_1- (dh12_1/F12_1)*e12_1/(beta2_1*h12_1^2);
    
    v11_1=(dh11_1/F11_1)*(dbeta1_1*e11_1^2)/(beta1_1^2*h11_1^2);
    v12_1=(dh12_1/F12_1)*(dbeta2_1*e12_1^2)/(beta2_1^2*h12_1^2);
    
    mu21_1=1/h21_1-dh21_1*x21_1/h21_1^2;
    mu22_1=1/h22_1-dh22_1*x22_1/h22_1^2;            %%% (22)
    
    mu1_1=[mu11_1  0; 0    mu12_1];
    R2_1=[h21_1  0; 0  h22_1];
    v1_1=[v11_1;v12_1];
    mu2_1=[mu21_1 0; 0 mu22_1];
    
    %%%%%%%%%%  control design
    phi_1=norm(mu1_1)^2*(norm(R2_1)^2+norm(qd)^2)+norm(v1_1)^2;
    z1_1=[xi11_1; xi12_1];
    alpha1_1=-(k1_1+r1_1*phi_1)*z1_1;                %%%%%%%%% (35)
    
   
   
    
    dalpha2f_1=1/w1_1 *(inv(mu1_1*R2_1)*alpha1_1-alpha2f_1);
    alpha2f_1=alpha2f_1+dalpha2f_1*step;
    
    
    xi2_1=[xi21_1; xi22_1];
    z2_1=xi2_1-alpha2f_1;
    Z_1=[ q1_1; q2_1;qd1;qd2;h11_1;h12_1;h21_1;h22_1];
    
    for j=1:hidden2_2
        Sz(j)=exp(-((norm(Z_1-NNa(:,j)))^2)/NNb^2);
    end
    Phi2_1=norm(Sz);
    
    dhata_1=-r3_1*hata_1+r4_1*Phi2_1*norm(z2_1)^2;
    hata_1=hata_1+dhata_1*step;
    
    u_1=-inv(mu2_1)*(k2_1*z2_1+(mu1_1*R2_1)'*z1_1+r2_1*hata_1*Phi2_1*z2_1);
    
    u1_1=u_1(1,1); u2_1=u_1(2,1);
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% system model
    M11_1=m1*l1^2+m2*(l1^2+l2^2+2*l1*l2*cos(q2_1))+I1+I2;
    M12_1=m2*(l2^2+l1*l2*cos(q2_1))+I2;
    M22_1=m2*l2^2+I2;
    C11_1=-m2*l1*l2*dq2_2*sin(q2_1);
    C12_1=-m2*l1*l2*(dq1_1+dq2_1)*sin(q2_1);
    C21_1=m2*l1*l2*dq1_1*sin(q2_1);
    C22_1=0;
    G1_1=(m1*l2+m2*l1)*ge*cos(q1_1)+m2*l2*ge*cos(q1_1+q2_1);
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
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q1_1s(i)=q1_1; qd1s(i)=qd1;
    q2_1s(i)=q2_1;qd2s(i)=qd2;
    
    upbound1_1=F11_1+qd1; downbound1_1=-F11_1+qd1; upbound2_1=F12_1+qd2; downbound2_1=-F12_1+qd2;
    upbound1s_1(i)=upbound1_1; downbound1s_1(i)=downbound1_1; upbound2s_1(i)=upbound2_1; downbound2s_1(i)=downbound2_1;
    dq1_1s(i)=dq1_1; dq2_1s(i)=dq2_1;
 
    
    upboundpra1_1=sqrt(F11_1^2*beta1_1^2+F12_1^2*beta2_1^2); downboundpra1_1=-sqrt(F11_1^2*beta1_1^2+F12_1^2*beta2_1^2);
    upboundpra1s_1(i)=upboundpra1_1; downboundpra1s_1(i)=downboundpra1_1;
    
    e_1=[e11_1;e12_1];
    es_1(i)=norm(e_1);
    

    u1_1s(i)=u1_1;
    u2_1s(i)=u2_1;
    hata_1s(i)=hata_1;
    
end



figure(1);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),es_1,'b',t(1:n), upboundpra1s_1,'g--',t(1:n), downboundpra1s_1,'g--', 'linewidth',2.2);
h=legend({'$\epsilon=0.04$','boundary curve'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$||e||$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);

figure(2);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),es_2,'b',t(1:n), upboundpra1s_2,'g--',t(1:n), downboundpra1s_2,'g--', 'linewidth',2.2);
h=legend({'$\epsilon=0.004$','boundary curve'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$||e||$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);

figure(3);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),u1_1s,'b','linewidth',2);
h=legend({'$\epsilon=0.04$'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$u_1$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);

figure(4);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),u1_2s,'b','linewidth',2);
h=legend({'$\epsilon=0.004$'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$u_1$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);


figure(5);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),u2_1s,'b','linewidth',2);
h=legend({'$\epsilon=0.04$'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$u_2$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);


figure(6);
h=gca;
set(h,'FontSize',18);
propertyeditor('on');
plot(t(1:n),u2_2s,'b','linewidth',2);
h=legend({'$\epsilon=0.004$'},'interpreter','latex');
xlabel('Time(sec)','FontName','Times New Roman','FontSize',16);
ylabel('$u_2$','interpreter','latex','linewidth',20);
set(h,'Fontsize',16);


