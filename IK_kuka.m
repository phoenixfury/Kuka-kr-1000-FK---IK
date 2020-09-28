function Q1 = IK_kuka(T)
%Get the inverse kinematics of the kuka robot
%   We will get the solution analytically by dividing the problem into 2
%   parts, the 1st part is the position & the 2nd is the rotation

%The position part
y = T(2,4); 
x = T(1,4);

% handle if atan2(y,x0 == 0) if it's zero then let q1 = 30 degrees
if atan2(y,x)==0
    q1 = pi/6;
else
    q1 = atan2(y,x);
end

%now get q3 to get q2
a1 = 400;
a2 = 560;
a4 = 515;
x_new = sqrt(x^2 + y^2); 
y_new = -T(3,4)+a1;

%first q3 solution
q3 = acos( (x_new^2 + y_new^2 - a2^2 - a4^2) / (2 * a2 *a4) );

%Get q2 now
%check if q2 > 0 or q2 < 0
if q3 > 0
    m = -1;
else
    m = 1;
end

q2 = m * atan( a4*sin(q3) / (a2 + a4*cos(q3) )) + atan( y_new/x_new );

%Now we have our 3 position angles we plug them back into the forward
%kinematics to get R0_3
T0_3 = FK_kuka([q1,q2,q3]);
%Replace inv(A)*b with A\b
T3_6 = T0_3 \ T;

% Extract and inspect the first component
% if the first element is 1 or -1 there is no unique solution
% so we have to assume one of the 2 angles
% q4 + q6 = atan2(T3_6(2,3),T3_6(2,2))
if T3_6(1,1) == 1
    q5 = acos(T3_6(1,1));
    % let q6 = pi/8
    q6 = pi/8;
    q4 = (atan2(T3_6(2,3),T3_6(2,2)) -  q6) ;
% q4 - q6 = atan2(T3_6(2,3),T3_6(2,2))
elseif T3_6(1,1) == -1
    q5 = acos(T3_6(1,1));
    % let q6 = pi/6
    q6 = pi/6;
    q4 = q6 + atan2(T3_6(2,3),T3_6(2,2));
%Else we have 2 pairs of solutions        
else
    q4 = atan2( T3_6(2,1), -T3_6(3,1));
    q6 = atan2( T3_6(1,2), T3_6(1,3));
    if T3_6(3,1) ~= 0
       q5 =  atan2( (T3_6(1,3)/(cos(q6))), T3_6(1,1));
    else
        q5 =  atan2( (T3_6(1,3)/(sin(q6))), T3_6(1,1));
    end
end
Q1 = [q1 q2 q3 q4 q5 q6];
end

