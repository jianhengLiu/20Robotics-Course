syms l0 l1 l2 theta1 theta2 theta3 theta4 theta5 theta6 real

w1=[0;0;1];
w2=[0;1;0];w6=w2;
w3=[-1;0;0];w4=w3;w5=w3;

q1=[0;0;0];
q2=[0;0;l0];
q3=[0;0;l0];
q4=[0;l1;l0];
q5=[0;l1+l2;l0];
q6=q2;


v1 = -cross(w1,q1);
s1=[v1;w1];
v2 = -cross(w2,q2);
s2=[v2;w2];
v3 = -cross(w3,q3);
s3=[v3;w3];
v4 = -cross(w4,q4);
s4=[v4;w4];
v5 = -cross(w5,q5);
s5=[v5;w5];
v6 = -cross(w6,q6);
s6=[v6;w6];

expw1=angvec2r(theta1,w1);
expw2=angvec2r(theta2,w2);
expw3=angvec2r(theta3,w3);
expw4=angvec2r(theta4,w4);
expw5=angvec2r(theta5,w5);
expw6=angvec2r(theta6,w6);


exps1=[expw1,(eye(3)-expw1)*cross(w1,v1)+w1*w1'*v1*theta1;0,0,0,1];
exps2=[expw2,(eye(3)-expw2)*cross(w2,v2)+w2*w2'*v2*theta2;0,0,0,1];
exps3=[expw3,(eye(3)-expw3)*cross(w3,v3)+w3*w3'*v3*theta3;0,0,0,1];
exps4=[expw4,(eye(3)-expw4)*cross(w4,v4)+w4*w4'*v4*theta4;0,0,0,1];
exps5=[expw5,(eye(3)-expw5)*cross(w5,v5)+w5*w5'*v5*theta5;0,0,0,1];
exps6=[expw6,(eye(3)-expw6)*cross(w6,v6)+w6*w6'*v6*theta6;0,0,0,1];


q2d = expsMultiplyVector(exps1,q2);
q2d = simplify(q2d)
q3d = expsMultiplyVector(exps1*exps2,q3);
q3d = simplify(q3d)
q4d = expsMultiplyVector(exps1*exps2*exps3,q4);
q4d = simplify(q4d)
q5d = expsMultiplyVector(exps1*exps2*exps3*exps4,q5);
q5d = simplify(q5d)
q6d = expsMultiplyVector(exps1*exps2*exps3*exps4*exps5,q6);
q6d = simplify(q6d)


w2d=expw1*w2;
w2d=simplify(w2d)
w3d=expw1*expw2*w3;
w3d=simplify(w3d)
w4d=expw1*expw2*expw3*w4;
w4d=simplify(w4d)
w5d=expw1*expw2*expw3*expw4*w5;
w5d=simplify(w5d)
w6d=expw1*expw2*expw3*expw4*expw5*w6;
w6d=simplify(w6d)


s1d=lidaishu(w1,q1)

s2d=lidaishu(w2d,q2d);
s2d=simplify(s2d)
s3d=lidaishu(w3d,q3d);
s3d=simplify(s3d)
s4d=lidaishu(w4d,q4d);
s4d=simplify(s4d)
s5d=lidaishu(w5d,q5d);
s5d=simplify(s5d)
s6d=lidaishu(w6d,q6d);
s6d=simplify(s6d)

Jst_s=[s1d,s2d,s3d,s4d,s5d,s6d]

gst0 = [eye(3),[0;l1+l2;l0];0,0,0,1];
gst = exps1*exps2*exps3*exps4*exps5*exps6*gst0;
gst = simplify(gst);
Adj_gstinv = Adjoint_ginv(gst);
Jst_b = Adj_gstinv * Jst_s;
Jst_b = simplify(Jst_b)

