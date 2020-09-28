function rot_y = Rot_y(q)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
rot_y = [ cos(q), 0, sin(q), 0;
          0, 1, 0, 0;
          -sin(q), 0, cos(q), 0;
          0, 0, 0, 1];
end

