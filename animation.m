clc;

l0=20;l1=15;l2=15;le=5;
c=[l0,l1,l2,le];

% Trajectory
N=100;
x=zeros(1,N);
y=linspace(5,25,N);
phi=linspace(pi,pi,N);

% Joint space
for i=1:N
    p=[x(i),y(i),phi(i)];
    joint(:,i)=IK(p,c);
end


for i=1:N
T=FK(c,joint(:,i));

% Manipulator
plot3(x,y,zeros(1,N),'b')
hold on;
for j=1:3
    pj=T{j}(1:3,4);
    pj1=T{j+1}(1:3,4);
    plot3([pj(1),pj1(1)],[pj(2),pj1(2)],[pj(3),pj1(3)],'k','LineWidth',4);
end

% End-effector
pe=T{4}(1:3,4);
pe1=T{5}(1:3,4);
plot3([pe(1),pe1(1)],[pe(2),pe1(2)],[pe(3),pe1(3)],'b','LineWidth',4);

% Frames
for j=1:5
    pj=T{j}(1:3,4); % origin
    Rj=T{j}(1:3,1:3); % rotation matrix
    
    scale=3;
    xj=pj + Rj(:,1)*scale;
    yj=pj + Rj(:,2)*scale;
    zj=pj + Rj(:,3)*scale;

    plot3([pj(1),xj(1)],[pj(2),xj(2)],[pj(3),xj(3)],'g','LineWidth',2); % x
    plot3([pj(1),yj(1)],[pj(2),yj(2)],[pj(3),yj(3)],'m','LineWidth',2); % y
    plot3([pj(1),zj(1)],[pj(2),zj(2)],[pj(3),zj(3)],'r','LineWidth',2); % z
end

% Base
plot3([0,0],[0,0],[-10,10],'Color',[0.3,0.3,0.3,0.6],'LineWidth',5);


xlabel('x');ylabel('y');zlabel('z');zlim([-10,30]);xlim([-10,30]);ylim([-10,30]);
grid on;

hold off

drawnow % draws each frame

end