function t = IK(pos,c)

l0 = c(1);
l1 = c(2);
l2 = c(3);
le = c(4);

x=pos(1)-l0; %x = T0e(1,4) - l0;
y=pos(2); %y = T0e(2,4);
phi=pos(3); %z = T0e(3,4);

x_d=x-le*cos(phi);
y_d=y-le*sin(phi);
gamma=atan2(-y_d/sqrt(x_d^2 +y_d^2),-x_d/sqrt(x_d^2 +y_d^2));

a=2*l1*x_d;b=2*l1*y_d;c=x_d^2 + y_d^2 +l1^2 - l2^2;

t1=[atan2(sqrt(a^2 + b^2 - c^2),c) + atan2(b,a),atan2(-sqrt(a^2 + b^2 - c^2),c) + atan2(b,a)];

t11=[];
for i=1:2
    if t1(i)<=pi && t1(i)>=-pi
        t11=[t11,t1(i)];
    end
end

t2 = atan2((y_d-l1.*sin(t11))./l2,(x_d-l1.*cos(t11))./l2) - t11;
for i=1:1
    if t2(i)>pi
        t2(i)=t2(i)-2*pi;
    elseif t2(i)<-pi
        t2(i)=t2(i)+2*pi;
    end
end

t3 = phi-t11-t2;
for i=1:1
    if t3(i)>pi
        t3(i)=t3(i)-2*pi;
    elseif t3(i)<-pi
        t3(i)=t3(i)+2*pi;
    end
end

t=[t11;t2;t3];
end