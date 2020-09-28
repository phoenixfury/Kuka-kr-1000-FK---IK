%% Define the forward kinematics

syms q1 q2 q3 q4 q5 q6 a1 a2 a4

%Translation part
T0_1 = Rot_z(q1);
T1_2 =Tr_z(a1)*Rot_y(q2) ;
T2_3 = Tr_x(a2)*Rot_y(q3);
T3_4 = Tr_x(a4);

%Rotation part
T4_5_6 = Rot_x(q4) * Rot_y(q5) * Rot_x(q6);

T0_6 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5_6;
%T0_6 = simplify(T0_6)

T4_5_6 = simplify(T4_5_6)
T0_3 = T0_1 * T1_2 * T2_3 * T3_4
