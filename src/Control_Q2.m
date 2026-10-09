Q2 = tf(6 , [6 54 48 0 ]);
% rlocus(Q2);
%sgrid(0.707 , 2.828); % sigma = 1/Time c, sigma = Wn*Zeta
PD = pid(40.12 , 0 , 20.06)
Gp = feedback(PD*Q2 , 1)
rlocus(Gp)
sgrid(0.707 , 2.828)
stepinfo(Gp)
