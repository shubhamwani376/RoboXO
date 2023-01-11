function T = FK(c,joint)

l0 = c(1);
l1 = c(2);
l2 = c(3);
le = c(4);
t1 = joint(1);
t2 = joint(2);
t3 = joint(3);
%t5 = joint(5);
%t6 = joint(6);
g=9.8;mM=0.1;rho=9*1.24;%g/cm
m1=rho*l1/1000;m2=rho*l2/1000;m3=rho*le/1000;mE=0.1;

Torque_max = g*(m1*l1/2 + m2*(l1+ 0.5*l2) + m3*(l1+l2+ le/2)+ mM*l1 + mM*(l1+l2) + mE*(l1+l2+le))/100;

%l0=0;
% DH parameters
DH = [0 l0 0 t1;0 l1 0 t2;0 l2 0 t3;0 le 0 0];%;-pi/2 0 0 t5;pi/2 0 0 t6;0 0 de 0];
alpha = DH(:,1); a = DH(:,2); d = DH(:,3); theta = DH(:,4);

% initial
To = eye(4);
fx = 0; fy = 0; fz = 0;
T{1} = To;

for j = 1:4

    Ti = [cos(theta(j)) -sin(theta(j)) 0 a(j);
        sin(theta(j))*cos(alpha(j)) cos(theta(j))*cos(alpha(j)) ...
        -sin(alpha(j)) -sin(alpha(j))*d(j);
        sin(theta(j))*sin(alpha(j)) cos(theta(j))*sin(alpha(j)) ...
        cos(alpha(j)) cos(alpha(j))*d(j);0 0 0 1];
    To = To*Ti;
    fx = [fx To(1,4)]; % frame x coordiante
    fy = [fy To(2,4)]; % frame y coordiante
    fz = [fz To(3,4)]; % frame z coordiante
    T{j+1} = To;

end

end