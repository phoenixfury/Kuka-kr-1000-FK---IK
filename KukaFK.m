function H = KukaFK(varargin)
%KUKAFK Gives the FK for translation or the full FK if given 3 angles or 6
%angles
%   %Get the forward kinematics by using the angles
%   use the q vector to get the orientation and position of the kuka robot


%Define the constants here
d1 = 25;
a1 = 400; 
a2 = 560;
d3 = 25;
a3 = 515;
delta_q3 = atan2(d3,a3);
d3_dash = sqrt(d3^2 + a3^2);
%Translation part
%return the first 3 joints forward kinematics for further use in inverse
%kinematics
H_transl = Rz(varargin{1}(1)) * Tx(d1) * Tz(a1) * Ry(varargin{1}(2)) * Tz(a2) * Ry(varargin{1}(3) - delta_q3) * Tx(d3_dash)* Ry(delta_q3);

if length(varargin{1}) == 3
    H = H_transl;
else
    %Return the full forward kinematics matrix
    H_rotation =  Rx(varargin{1}(4)) * Ry(varargin{1}(5)) * Rx(varargin{1}(6));
    H = H_transl * H_rotation;
    
end
end

