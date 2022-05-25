function out1 = sonar2NEDtest(in1,in2,in3,in4)
%sonar2NED
%    OUT1 = sonar2NEDtest(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    18-May-2022 10:24:52

d_x = in3(1,:);
d_y = in3(2,:);
d_z = in3(3,:);
epsilon1 = in2(:,2);
epsilon2 = in2(:,3);
epsilon3 = in2(:,4);
eta = in2(:,1);
ps_x = in1(1,:);
ps_y = in1(2,:);
ps_z = in1(3,:);
x = in4(:,1);
y = in4(:,2);
z = in4(:,3);
t2 = epsilon1.^2;
t3 = epsilon2.^2;
t4 = epsilon3.^2;
t5 = epsilon1.*epsilon2.*2.0;
t6 = epsilon1.*epsilon3.*2.0;
t7 = epsilon2.*epsilon3.*2.0;
t8 = epsilon1.*eta.*2.0;
t9 = epsilon2.*eta.*2.0;
t10 = epsilon3.*eta.*2.0;
t11 = t2.*2.0;
t12 = t3.*2.0;
t13 = t4.*2.0;
t14 = -t8;
t15 = -t9;
t16 = -t10;
t17 = t5+t10;
t18 = t6+t9;
t19 = t7+t8;
t20 = t5+t16;
t21 = t6+t15;
t22 = t7+t14;
t23 = t11+t12-1.0;
t24 = t11+t13-1.0;
t25 = t12+t13-1.0;
out1 = [ps_x-d_x.*t25+d_y.*t20+d_z.*t18-t25.*x-t20.*y-t18.*z;ps_y+d_x.*t17-d_y.*t24+d_z.*t22+t17.*x+t24.*y-t22.*z;ps_z+d_x.*t21+d_y.*t19-d_z.*t23+t21.*x-t19.*y+t23.*z;1.0];