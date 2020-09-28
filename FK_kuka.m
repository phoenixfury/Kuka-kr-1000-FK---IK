function T = FK_kuka(varargin)
%Get the forward kinematics by using the angles
%   use the q vector to get the orientation and position of the kuka robot

%Define the constants here
a1 = 400;
a2 = 560;
a4 = 515;

%Translation part
T0_1 = Rot_z(varargin{1}(1));
T1_2 =Tr_z(a1)*Rot_y(varargin{1}(2)) ;
T2_3 = Tr_x(a2)*Rot_y(varargin{1}(3));
T3_4 = Tr_x(a4);

%return the first 3 joints forward kinematics for further use in inverse
%kinematics
T_fk_pos = T0_1 * T1_2 * T2_3 * T3_4;
if length(varargin{1}) == 3
    T = T_fk_pos;
else
    %Rotation part
    T4_5_6 = Rot_x(varargin{1}(4)) * Rot_y(varargin{1}(5)) * Rot_x(varargin{1}(6));

    %Return the full forward kinematics matrix
    T_fk_full = T_fk_pos * T4_5_6;
    T = T_fk_full;
    
end
end

