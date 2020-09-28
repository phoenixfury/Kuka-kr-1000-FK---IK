%% This is to test the FK and the IK functions
q = [ pi/4 pi/6 pi/3 pi/8 pi/4 pi/4]

fk = FK_kuka(q)

g = IK_kuka(fk)

fk_check = FK_kuka(g)

% Test the singularity of q5

q2 =  [ pi/4 pi/6 pi/3 pi/8 0 pi/4]

fk2 = FK_kuka(q2)

g2 = IK_kuka(fk2)

fk_check2 = FK_kuka(g2)
