%% Testing the forward kinematics
% Home Position
q = [0 0 0 0 0 0];
Home = KukaFK(q)

% Another position
q = [ pi/4 pi/7 pi/6 pi/3 pi/2 pi/4];
pos1 = KukaFK(q)

%% Testing the inverse kinematics
q_pos1 = KukaIK(pos1)
sol1 = KukaFK(q_pos1(1,:))
sol2 = KukaFK(q_pos1(2,:))
sol3 = KukaFK(q_pos1(3,:))
sol4 = KukaFK(q_pos1(4,:))
sol5 = KukaFK(q_pos1(5,:))
sol6 = KukaFK(q_pos1(6,:))
sol7 = KukaFK(q_pos1(7,:))
sol8 = KukaFK(q_pos1(8,:))


%% Workspace limit violation pos

q = [ pi/4 pi/2 -pi/3 pi/4 -pi/5 pi/4];
viol = KukaFK(q)
inv_sol = KukaIK(viol)

%% singularity violation

q = [ pi/4 pi/4 pi/4 pi/3 0 pi/5]
sing = KukaFK(q)
inv_sol = KukaIK(sing)

%% 100 random test
disp('Test FK IK in 100 random configurations')
for i=1:100
    %     q_t = rand(1,6);  
    a = [-pi -pi 0 -pi -pi -pi];
    b = [pi pi 50 pi pi pi];
    q_t = (b-a).*rand(1,6) + a;    
    T=KukaFK(q_t);
    q_new=KukaIK(T);
    for j=1:size(q_new,1) % check for all IK solutions
        Tik=KukaFK(q_new(j,:));
        if norm(T-Tik)>1e-3
            q_new(j,:)
            disp('Found IK error!!!')
        end
    end
end
disp('Test ended')

