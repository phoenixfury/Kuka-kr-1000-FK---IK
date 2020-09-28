%% Inverse kinematics

%% position solve

% x = cos(q1) * (l2 cos(q2) + l3 cos(q2+q3))
% y = sin(q1) * (l2 cos(q2) + l3 cos(q2+q3))
% z = l1 - l2 sin(q2) - l3 sin(q2+q3))

% q1 = atan2(y,x), y = T0_3(2,4), x = T0_3(1,4)

% sqrt(x^2 + y^2) = l2 cos(q2) + l3 cos(q2+q3) = x_new
%          -z+ l1 = l2 sin(q2) + l3 sin(q2+q3))= y_new

%q3 = acosd( (x_new^2 + y_new^2 - l2^2 - l3^2) / (2 * l2 *l3) )

%q2 = m * atand( l3*sin(q3) / (l2 + l3*cos(q3) )) + atand( y_new/x_new )


%% Rotation solve
% if nx ~= 1, q4 = atan2( ny, -nz)
% q6 = atan2( sx, ax)

% if ax ~= 0 q5 = atan2 ( ax/ cosq6, nx)
% else q5 = atan2 ( ax/ sinq6, nx)

% if nx == 1 then q5 = acos(nx) 