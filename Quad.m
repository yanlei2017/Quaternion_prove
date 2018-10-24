clear all；
Q0=[0,0,0,1];%初始位置x=0,y=0,z=1
Q0_n=quatnormalize(Q0);%单位化

yaw1=deg2rad(0);
pitch1=deg2rad(0);
roll1=deg2rad(60);%绕X轴转动+60度
dcm=angle2dcm(yaw1,pitch1,roll1,'ZYX');
Q0_dcm=dcm.*[0,0,1];
q1= angle2quat(yaw1, pitch1, roll1,'ZYX');
q1_=quatconj(q1);%共轭，单位四元数的共轭和逆相等，因为A*=A_ / ||A||,表示旋转的四元数都是单位四元数
%q1_=quatinv(q1)
Q1=quatmultiply(quatmultiply(q1,Q0),q1_);%后3个元素代表位置

yaw2=deg2rad(0);
pitch2=deg2rad(0);
roll2=deg2rad(-60);%绕X轴转动-60度,转回去了

 q2= angle2quat(yaw2, pitch2, roll2,'ZYX');
%q2= angle2quat(roll2, pitch2, yaw2,'XYZ');
q2_=quatconj(q2);

Q2=quatmultiply(quatmultiply(q2,Q1),q2_);%Q2应该等于Q0
 
q3=quatmultiply(q1,q2);%2步四元数旋转合为一步
q3_=quatconj(q3);
Q3=quatmultiply(quatmultiply(q3,Q0),q3_);%Q3应该等于Q2、Q0
 
