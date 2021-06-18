P1 = [0;0];
P2 = [2;1];
P3 = [3;3];

acceleration_b = [6;6];

td12 = 1; 
td23 = 1;

% subplot(3,1,1)
plot3(0,P1(1),P1(2),'r*');
hold on;
plot3(td12,P2(1),P2(2),'r*');
hold on;
plot3(td12+td23,P3(1),P3(2),'r*');
hold on;

acceleration_1 = sign(P2 - P1).*acceleration_b;
tb1 = td12 - sqrt(td12^2-2*(P2-P1)./acceleration_1)
velocity12 = (P2-P1)./(td12-tb1/2)

acceleration_3 = sign(P2 - P3).*acceleration_b
tb3 = td23 - sqrt(td23^2 + 2*(P3-P2)./acceleration_3)
velocity23 = (P3-P2)./(td23-tb3/2)

acceleration_2 = sign(velocity23-velocity12).*acceleration_b
tb2 = (velocity23-velocity12)./(acceleration_2)

t23 = td23 - tb2/2 - tb3

t12 = td12 - tb2/2 - tb1

t_b1x = 0:0.001:tb1(1);
p_b1x = P1(1) + 1/2*acceleration_1(1)*t_b1x.^2;
v_b1x = acceleration_1(1)*t_b1x;
a_b1x = acceleration_1(1) + 0*t_b1x;
tx = t_b1x;
px = p_b1x;
vx = v_b1x;
ax = a_b1x;

t_b1y = 0:0.001:tb1(2);
p_b1y = P1(2) + 1/2*acceleration_1(2)*t_b1y.^2;
v_b1y = acceleration_1(2)*t_b1y;
a_b1y = acceleration_1(2) + 0*t_b1y;
ty = t_b1y;
py = p_b1y;
vy = v_b1y;
ay = a_b1y;

t_12x = 0:0.001:t12(1);
p_12x = px(end) + velocity12(1).*t_12x;
v_12x = vx(end) + 0 * t_12x;
a_12x = 0*t_12x;
tx = [tx t_12x+tx(end)];
px = [px p_12x];
vx = [vx v_12x];
ax = [ax a_12x];

t_12y = 0:0.001:t12(2);
p_12y = py(end) + velocity12(2).*t_12y;
v_12y = vy(end) + 0 * t_12y;
a_12y = 0*t_12y;
ty = [ty t_12y+ty(end)];
py = [py p_12y];
vy = [vy v_12y];
ay = [ay a_12y];

t_b2x = 0:0.001:tb2(1);
p_b2x = px(end) + vx(end)*t_b2x + 1/2*acceleration_2(1)*t_b2x.^2;
v_b2x = vx(end) + acceleration_2(1)*t_b2x;
a_b2x = acceleration_2(1) +0*t_b2x;
tx = [tx t_b2x+tx(end)];
px = [px p_b2x];
vx = [vx v_b2x];
ax = [ax a_b2x];

t_b2y = 0:0.001:tb2(2);
p_b2y = py(end) + vy(end)*t_b2y + 1/2*acceleration_2(2)*t_b2y.^2;
v_b2y = vy(end) + acceleration_2(2)*t_b2y;
a_b2y = acceleration_2(2) +0*t_b2y;
ty = [ty t_b2y+ty(end)];
py = [py p_b2y];
vy = [vy v_b2y];
ay = [ay a_b2y];

t_23x = 0:0.001:t23(1);
p_23x = px(end) + velocity23(1)*t_23x;
v_23x = vx(end) + 0*t_23x;
a_23x = 0*t_23x;
tx = [tx t_23x+tx(end)];
px = [px p_23x];
vx = [vx v_23x];
ax = [ax a_23x];

t_23y = 0:0.001:t23(2);
p_23y = py(end) + velocity23(2)*t_23y;
v_23y = vy(end) + 0*t_23y;
a_23y = 0*t_23y;
ty = [ty t_23y+ty(end)];
py = [py p_23y];
vy = [vy v_23y];
ay = [ay a_23y];

t_b3x = 0:0.001:tb3(1);
p_b3x = px(end) + vx(end)*t_b3x + 1/2*acceleration_3(1)*t_b3x.^2;
v_b3x = v_23x(end) + acceleration_3(1)*t_b3x;
a_b3x = acceleration_3(1) + 0*t_b3x;
tx = [tx t_b3x+tx(end)];
px = [px p_b3x];
vx = [vx v_b3x];
ax = [ax a_b3x];

t_b3y = 0:0.001:tb3(2);
p_b3y = py(end) + vy(end)*t_b3y + 1/2*acceleration_3(2)*t_b3y.^2;
v_b3y = v_23y(end) + acceleration_3(2)*t_b3y;
a_b3y = acceleration_3(2) + 0*t_b3y;
ty = [ty t_b3y+ty(end)];
py = [py p_b3y];
vy = [vy v_b3y];
ay = [ay a_b3y];

t = 0:0.001:2.001;
plot3(t,px,py,'b')
xlabel('t')
ylabel('x')
zlabel('y')
grid on
title('position')
% subplot(2,2,1)
% plot(t,vx,'b')
% xlabel('t')
% ylabel('vx')
% grid on
% title('velocity-x')
% subplot(2,2,2)
% plot(t,vy,'b')
% xlabel('t')
% ylabel('vy')
% grid on
% title('velocity-y')
% subplot(2,2,3)
% plot(t,ax,'b')
% xlabel('t')
% ylabel('ax')
% grid on
% title('acceleration-x')
% subplot(2,2,4)
% plot(t,ay,'b')
% xlabel('t')
% ylabel('ay')
% grid on
% title('acceleration-y')