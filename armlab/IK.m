%% MOD-DH参数
%连杆偏移       %连杆长度       %连杆扭角
d1 = robot.d(1);a1 = robot.a(1);alpha1 = robot.alpha(1);
d2 = robot.d(2);a2 = robot.a(2);alpha2 = robot.alpha(2);
d3 = robot.d(3);a3 = robot.a(3);alpha3 = robot.alpha(3);
d4 = robot.d(4);a4 = robot.a(4);alpha4 = robot.alpha(4);
d5 = robot.d(5);a5 = robot.a(5);alpha5 = robot.alpha(5);
d6 = robot.d(6);a6 = robot.a(6);alpha6 = robot.alpha(6);

%%
nx=a(1,1);ox=a(1,2);ax=a(1,3);px=a(1,4);
ny=a(2,1);oy=a(2,2);ay=a(2,3);py=a(2,4);
nz=a(3,1);oz=a(3,2);az=a(3,3);pz=a(3,4);

%j1
j1a = py - ay*d6;
j1b = px - ax*d6;

j11 = atan2(j1a,j1b)-atan2(-d2, sqrt(j1a^2+j1b^2-d2^2));
j12 = atan2(j1a,j1b)-atan2(-d2,-sqrt(j1a^2+j1b^2-d2^2));

% 这部分是进行一个判断，将小于1e-16的数字看作0，建议删除。
if j11<1e-16
   j11 = 0;
end
if j12<1e-16
   j12 = 0;
end

%j3
j3a1 = px*cos(j11) - d6*(ax*cos(j11) + ay*sin(j11)) + py*sin(j11) - a2;
j3b1 = pz - d1 - az*d6;
j3k1 = j3a1^2 + j3b1^2 - a4^2 - d4^2 - a3^2;
j3d1 = j3k1/(2*a3);
j3a2 = px*cos(j12) - d6*(ax*cos(j12) + ay*sin(j12)) + py*sin(j12) - a2;
j3b2 = pz - d1 - az*d6;
j3k2 = j3a2^2 + j3b2^2 - a4^2 - d4^2 - a3^2;
j3d2 = j3k2/(2*a3);

j31 = atan2(j3d1, sqrt(abs(a4^2 + d4^2 - j3d1^2))) - atan2(a4,d4);%j11
j32 = atan2(j3d1,-sqrt(abs(a4^2 + d4^2 - j3d1^2))) - atan2(a4,d4);%j11
j33 = atan2(j3d2, sqrt(abs(a4^2 + d4^2 - j3d2^2))) - atan2(a4,d4);%j12
j34 = atan2(j3d2,-sqrt(abs(a4^2 + d4^2 - j3d2^2))) - atan2(a4,d4);%j12

% 经过一系列运算后，最终得到J
J = [j11 j21 j31 j41 j51 j61;
     j11 j22 j32 j42 j52 j62;
     j12 j23 j33 j43 j53 j63;
     j12 j24 j34 j44 j54 j64;
     j11 j21 j31 j45 j55 j65;
     j11 j22 j32 j46 j56 j66;
     j12 j23 j33 j47 j57 j67;
     j12 j24 j34 j48 j58 j68];
    
