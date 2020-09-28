function rot_z = Rot_z(q)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
rot_z = [ cos(q), -sin(q), 0, 0;
          sin(q), cos(q), 0, 0;
          0, 0, 1, 0;
          0, 0, 0, 1];
end
