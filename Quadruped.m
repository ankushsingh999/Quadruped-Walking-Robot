
%% ----------------------- *** INITIAL DATA *** ------------------------%%
l1=.050;
l2=.070;
b=0.75;
v=0.1; % desired velocity of COG (m/sec)
u_b=(b/(1-b))*v; % average swing velocity of the leg wrt the body
u=v/(1-b); % average swing velocity of the leg wrt the ground
L=0.1; % Stride Length which is the amount of COG displacement in one cycle time.
T= L/v; % Cycle Period
Tt=(1-b)*T;
Ts=b*T;
%% ------------------- *** GAIT GENERATION *** ---------------------- %%
% b=.9
p(1)=0; % Kinematic phase of leg 1
p(2)=p(1)+1/2; % Kinematic phase of leg 2
p(3)=p(1)+b; % Kinematic phase of leg 3
p(4)=p(2)+b; % Kinematic phase of leg 4

%What if p(i) is equal or greater than 1?

    for i=1:4
        if p(i) >= 1
            p(i) = p(i) - 1;
        end
    end
p


%% ------------- *** POSITIONAL FOOT TRAJECTORY PLANNING *** ------------%%
%Transferring time is divided to 5 equal extents. Such a selection is completely
%arbitrary.
t0=0;
t1=Tt/5;
t2=2*t1;
t3=Tt-t2;
t4=Tt-t1;
t5=Tt;

Ttt=[t0,t1,t2,t3,t4,t5];

%%%%%%%%%%HERE EVERY THING IS IN Gi COORDINATE SYSTEM WHICH IS GROUND
%COORDINATE SYSTEM FOR EACH LEG
xdotmaxf_g= 1; %from the slides %maximum leg transferring speed.

zdotmaxf_g=xdotmaxf_g; % it is arbitrary.

xdotf_g=[0,0,xdotmaxf_g,xdotmaxf_g,0,0];

%Let's find horizonatal positions:
for i=1:6
    xf_g(i,1)=-L/2;
    xf_g(i,2)=xf_g(i,1) + 0;
    xf_g(i,3)=xf_g(i,2) + (t2-t1) * xdotmaxf_g / 2 ;
    xf_g(i,4)=xf_g(i,3) + (t3-t2) * xdotmaxf_g;
    xf_g(i,5)=xf_g(i,4) + (t4-t3) * xdotmaxf_g / 2 ;
    xf_g(i,6)=xf_g(i,5) + 0;
end


zdotf_g=[0,zdotmaxf_g,0,0,-zdotmaxf_g,0];

%Similarly for the vertical positions:
for i=1:6
    zf_g(i,1)=0;
    zf_g(i,2)=zf_g(i,1)+(t1-t0)*zdotmaxf_g/2;
    zf_g(i,3)=zf_g(i,2)+(t2-t1)*zdotmaxf_g/2;
    zf_g(i,4)=zf_g(i,3)+0;
    zf_g(i,5)=zf_g(i,4)-(t4-t3)*zdotmaxf_g/2;
    zf_g(i,6)=zf_g(i,5)-(t5-t4)*zdotmaxf_g/2;
end

%%%%%%%% Taking care of cooridinte systems

h=.1; %(m) height of the robot. Here h=l3 because it is assumed in the home positon.
D=l1+l2; %(m) Distance between the foot tip and hip joint from top view

alphaH=[-45*pi/180, 45*pi/180, -90*pi/180, 90*pi/180];

xb_g(1,1)= - (1-b) * L / 2; % frame b is a frame attached to the hip joint
xb_g(2,1)= - (1-b) * L / 2;
xb_g(3,1)= - (1-b) * L / 2;
xb_g(4,1)= - (1-b) * L / 2;


% x axis is parallel to the x axis of frame g (Gi).
zb_g(1,1)=h;
zb_g(2,1)=h;
zb_g(3,1)=h;
zb_g(4,1)=h;




dt=Tt/5;
xdotb_g=v;
zdotb_g=0;

for i=1:4
    for t=1:5 
        xb_g(i,t+1)=xb_g(i,t)+xdotb_g*dt;
        zb_g(i,t+1)=zb_g(i,t)+zdotb_g*dt;
    end
    for t=1:6
        xf_b(i,t)=xf_g(i,t)-xb_g(i,t);
        zf_b(i,t)=zf_g(i,t)-zb_g(i,t);
    end
end

% yf_b

yf_b=[0,0,0,0];

for i=1:4 % this is becaus of 6 legs

    for j=1:6 % time
    
        xf_H(i,j)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xf_b(i,j);yf_b(i);zf_b(i,j)];
        yf_H(i,j)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xf_b(i,j);yf_b(i);zf_b(i,j)];
        zf_H(i,j)=zf_b(i,j);
    end
end
  xf_H
  yf_H
  zf_H
% xf_b
% yf_b
% zf_b
xb_g
%% ----------------------- *** PLOT  DATA *** ---------------------%%
figure 

subplot(3,2,1)
plot(Ttt,xdotf_g)
xlabel('Time')
ylabel('Horizontal Velocity')

subplot(3,2,2)
plot(Ttt,zdotf_g)
xlabel('Time')
ylabel('Vertical Velocity')

subplot(3,2,3)
plot(Ttt,xf_g(4,:))
xlabel('Time')
ylabel('Horizontal Position')

subplot(3,2,4)
plot(Ttt,zf_g(4,:))
xlabel('Time')
ylabel('Vertical Position')

subplot(3,2,5)
plot(xf_g(4,:),zf_g(4,:))
xlabel('Horizontal Position')
ylabel('Vertical Position')

subplot(3,2,6)
plot(xf_b(4,:),zf_b(4,:))
xlabel('Horizontal Position wrt b')
ylabel('Vertical Position wrt b')

 
%% ---------------------------INVERSE KINEMATICS------------------------%%



for i=1:4 % for all 6 legs
    for j=1:6 % time

        L(i,j) = sqrt(xf_b(i,j).^2+zf_b(i,j).^2);
        theta2(i,j) = pi - acos((l1^2+l2^2-L(i,j)^2)/(2*l1*l2))
        alpha(i,j) = acos((L(i,j).^2+l1^2-l2^2)/(2*L(i,j)*l1));
        theta1(i,j) = atan2(zf_b(i,j),xf_b(i,j))-alpha(i,j)

    end
end

figure
for i = 1:4
    subplot(3,2,1)
plot(Ttt, theta1(i,:))
    xlabel('time')
    ylabel('theta1')
    subplot(3,2,2)
plot(Ttt, theta2(i,:))
    xlabel('time')
    ylabel('theta2')

end


%% -------------- *** VELOCITY FOOT TRAJECTORY PLANNING *** -------------%%

ydotf_b= 0;
for t=1:6
    xdotf_b(t)=xdotf_g(t)-xdotb_g;
    zdotf_b(t)=zdotf_g(t)-zdotb_g;
end


for j=1:6 % time
    for i=1:4 % leg numnber
        J(1,1)  = -l1*sin(theta1(i,j))-l2*sin(theta1(i,j)+theta2(i,j)) ;
        J(1,2)  =  -l2*sin(theta2(i,j)+theta1(i,j));
        J(2,1) = 0;
        J(2,2) = 0;
        J(3,1) = l1*cos(theta1(i,j)+l2*cos(theta1(i,j)+theta1(i,j)));
        J(3,2) = l2*cos(theta1(i,j)+theta2(i,j));
        Thetadot(i,j,:)=pinv(J)*[xdotf_b(j);ydotf_b;zdotf_b(j)];
    end
end

figure
for i = 1:4
    subplot(3,2,1)
    plot(Ttt,Thetadot(i,:,1) )
    xlabel('Time')
ylabel('thetadot1')
    subplot(3,2,2) 
    plot(Ttt,Thetadot(i,:,2) )
    xlabel('Time')
ylabel('thetadot2')

end

figure
for i = 1:6
    xlim([-0.07 0.05])
     ylim([-0.1 0.02])
    line1 = line([0,l1*cos(theta1(1,i))],[0,l1*sin(theta1(1,i))])
    line2 = line([l1*cos(theta1(1,i)),l1*cos(theta1(1,i))+l2*cos(theta1(1,i)+theta2(1,i))],[l1*sin(theta1(1,i)),l1*sin(theta1(1,i))+l2*sin(theta1(1,i)+theta2(1,i))])
    pause(1)
    delete(line1)
    delete(line2)
end
