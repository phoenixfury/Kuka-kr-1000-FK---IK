function q = KukaIK(H)
%KUKAIK IK solution for the kuka robot, this function will return 8
%solutions if there are no singularities
%   returns 8 solutions for the inverse kinematics if there are no
%   singularities, and it returns a 'singularity' if a singularity is found
%   and 'Work space limit error' if it exceeded the workspace limits
%% Constants
%Define the constants here
d1 = 25;
a1 = 400; 
a2 = 560;
d3 = 25;
a3 = 515;
delta_q3 = atan2(d3,a3);
d3_dash = sqrt(d3^2 + a3^2);

x = H(1,4);
y = H(2,4);
z = H(3,4);

%% The Translation problem
% The first solution
% Get q1 first
q1 = atan2( y, x);

dx = cos(q1) *d1;
dy = sin(q1) *d1;
d =  sqrt((x-dx)^2 + (y-dy)^2);
s = z - a1;
D = sqrt(d^2 + s^2);

q3_dash = acos( ( D^2 - a2^2 - d3_dash^2) / (2*a2*d3_dash));
limit = ( D^2 - a2^2 - d3_dash^2) / (2*a2*d3_dash);
if abs(limit) <= 1
    phi1 = atan2(s,d);
    phi2 = atan2(d3_dash*sin(q3_dash) ,(a2+d3_dash*cos(q3_dash)));

    q2_dash =  phi1 + phi2;

    %Get the real q2
    q2 = pi/2 - q2_dash;
    %Get the real q3
    q3 =  (q3_dash + delta_q3) - pi/2;
    
    
else
    disp('Robot limit reached in 1st sol');
    q1 = nan;
    q2 = nan;
    q3 = nan;
end

H1_trans = KukaFK([q1 q2 q3]);
%% The 2nd solution 'Translation'
% Get q1 first

if abs(limit) <= 1
    q1_2 = atan2( y, x);

    q2_2_dash = phi2 - phi1;

    q3_2 =  -((q3_dash - delta_q3) + pi/2);

    q2_2 = pi/2 + q2_2_dash;
else
    disp('Robot limit reached in 2nd sol');
    q1_2 = nan;
    q2_2 = nan;
    q3_2 =nan;
end

H2_trans = KukaFK([q1_2 q2_2 q3_2]);
%% The 3rd solution 'Translation'

q1_3 = (q1 + pi);
dx = cos(q1_3) *d1;
dy = sin(q1_3) *d1;
d =  sqrt((x-dx)^2 + (y-dy)^2);
s = z - a1;
D = sqrt(d^2 + s^2);
limit = ( D^2 - a2^2 - d3_dash^2) / (2*a2*d3_dash);
q3_3_dash = acos( ( D^2 - a2^2 - d3_dash^2) / (2*a2*d3_dash));
if abs(limit) <= 1
    phi1 = atan2(s,d);
    phi2 = atan2(d3_dash*sin(q3_3_dash) ,(a2+d3_dash*cos(q3_3_dash)));

    q2_3_dash =  phi1 + phi2;

    q2_3 = -(pi/2 - q2_3_dash);
    q3_3 = (- q3_3_dash + delta_q3 -pi/2) ;
else
    disp('Robot limit reached in 3rd sol');
    q1_3 = nan;
    q2_3 = nan;
    q3_3 =nan;
end

H3_trans = KukaFK([q1_3 q2_3 q3_3]);
%% The 4th solution 'Translation'
if abs(limit) < 1
    q1_4 = (q1 + pi);

    q2_4_dash = phi2 - phi1;

    q2_4 = -( pi/2 + q2_4_dash);
    q3_4 = (((q3_3_dash + delta_q3) - pi/2)); 
else
    disp('Robot limit reached in 4th sol');
    q1_4 = nan;
    q2_4 = nan;
    q3_4 =nan;
end
H4_trans = KukaFK([q1_4 q2_4 q3_4]);

%% 2nd part is the rotation problem
% We have 2 solutions in case of no singularities

H_rot_1 = inv(H1_trans)*H;
H_rot_2 = inv(H2_trans)*H;
H_rot_3 = inv(H3_trans)*H;
H_rot_4 = inv(H4_trans)*H;

%% The 2 solutions for H_rot_1

%nx=H_rot_1(1,1);
%ny=H_rot_1(2,1);
%nz=H_rot_1(3,1);

%sx=H_rot_1(1,2);
%sy=H_rot_1(2,2);
%sz=H_rot_1(3,2);

%ax=H_rot_1(1,3);
%ay=H_rot_1(2,3);
%az=H_rot_1(3,3);

if abs(H_rot_1(1,1)) ~= 1
    q4_1=atan2(H_rot_1(2,1),-H_rot_1(3,1));
    q4_2=atan2(-H_rot_1(2,1),H_rot_1(3,1));
    q6_1=atan2(H_rot_1(1,2),H_rot_1(1,3));
    q6_2=atan2(-H_rot_1(1,2),-H_rot_1(1,3));
    q5_1=atan2(sqrt(H_rot_1(1,3)^2 + H_rot_1(1,2)^2),H_rot_1(1,1));
    q5_2=atan2(-sqrt(H_rot_1(1,3)^2 + H_rot_1(1,2)^2),H_rot_1(1,1));
    if (q5_1 > -1e-8) && (q5_1 < 1e-8)
         disp('Singularity');
    end
    if (q5_2 > -1e-8) && (q5_2 < 1e-8)
         disp('Singularity');
    end
else
    disp('Singularity');
    q4_1=nan;
    q4_2=nan;
    q6_1=nan;
    q6_2=nan;
    q5_1=nan;
    q5_2= nan;
end
q_first = [q1 q2 q3 q4_1 q5_1 q6_1];
q_second = [q1 q2 q3 q4_2 q5_2 q6_2];

%% 2 solutions for H_rot_2

if abs(H_rot_2(1,1)) ~= 1
    q4_1=atan2(H_rot_2(2,1),-H_rot_2(3,1));
    q4_2=atan2(-H_rot_2(2,1),H_rot_2(3,1));
    q6_1=atan2(H_rot_2(1,2),H_rot_2(1,3));
    q6_2=atan2(-H_rot_2(1,2),-H_rot_2(1,3));
    q5_1=atan2(sqrt(H_rot_2(1,3)^2 + H_rot_2(1,2)^2),H_rot_2(1,1));
    q5_2=atan2(-sqrt(H_rot_2(1,3)^2 + H_rot_2(1,2)^2),H_rot_2(1,1));
    if (q5_1 > -1e-8) && (q5_1 < 1e-8)
         disp('Singularity');
    end
    if (q5_2 > -1e-8) && (q5_2 < 1e-8)
         disp('Singularity');
    end
else
    disp('Singularity');
    q4_1=nan;
    q4_2=nan;
    q6_1=nan;
    q6_2=nan;
    q5_1=nan;
    q5_2= nan;
end
q_third = [q1_2 q2_2 q3_2 q4_1 q5_1 q6_1];
q_fourth = [q1_2 q2_2 q3_2 q4_2 q5_2 q6_2];

%% 2 solutions for H_rot_3

if abs(H_rot_3(1,1)) ~= 1
    q4_1=atan2(H_rot_3(2,1),-H_rot_3(3,1));
    q4_2=atan2(-H_rot_3(2,1),H_rot_3(3,1));
    q6_1=atan2(H_rot_3(1,2),H_rot_3(1,3));
    q6_2=atan2(-H_rot_3(1,2),-H_rot_3(1,3));
    q5_1=atan2(sqrt(H_rot_3(1,3)^2 + H_rot_3(1,2)^2),H_rot_3(1,1));
    q5_2=atan2(-sqrt(H_rot_3(1,3)^2 + H_rot_3(1,2)^2),H_rot_3(1,1));
    if (q5_1 > -1e-8) && (q5_1 < 1e-8)
         disp('Singularity');
    end
    if (q5_2 > -1e-8) && (q5_2 < 1e-8)
         disp('Singularity');
    end
else
    disp('Singularity');
    q4_1=nan;
    q4_2=nan;
    q6_1=nan;
    q6_2=nan;
    q5_1=nan;
    q5_2= nan;
end
q_fifth = [q1_3 q2_3 q3_3 q4_1 q5_1 q6_1];
q_sixth = [q1_3 q2_3 q3_3 q4_2 q5_2 q6_2];

%% 2 solutions for H_rot_4

if abs(H_rot_4(1,1)) ~= 1
    q4_1=atan2(H_rot_4(2,1),-H_rot_4(3,1));
    q4_2=atan2(-H_rot_4(2,1),H_rot_4(3,1));
    q6_1=atan2(H_rot_4(1,2),H_rot_4(1,3));
    q6_2=atan2(-H_rot_4(1,2),-H_rot_4(1,3));
    q5_1=atan2(sqrt(H_rot_4(1,3)^2 + H_rot_4(1,2)^2),H_rot_4(1,1));
    q5_2=atan2(-sqrt(H_rot_4(1,3)^2 + H_rot_4(1,2)^2),H_rot_4(1,1));
    if (q5_1 > -1e-8) && (q5_1 < 1e-8)
         disp('Singularity');
    end
    if (q5_2 > -1e-8) && (q5_2 < 1e-8)
         disp('Singularity');
    end
else
    disp('Singularity');
    q4_1=nan;
    q4_2=nan;
    q6_1=nan;
    q6_2=nan;
    q5_1=nan;
    q5_2= nan;
end
q_seventh = [q1_4 q2_4 q3_4 q4_1 q5_1 q6_1];
q_eigth = [q1_4 q2_4 q3_4 q4_2 q5_2 q6_2];

q = [ q_first; 
      q_second;
      q_third;
      q_fourth;
      q_fifth;
      q_sixth;
      q_seventh;
      q_eigth;];
end

