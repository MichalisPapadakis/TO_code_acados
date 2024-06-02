function yt = leg_dynamics_f(x,u_gen,D)
%This function is used to formulate the RK4 discrete dynamics in 
%`leg_model_discrete`

import casadi.*

q  = x(1:5 );
qt = x(6:10);

Q1 = q(1);Q1t = qt(1);
Q2 = q(2);Q2t = qt(2);
Q3 = q(3);Q3t = qt(3);
Q4 = q(4);Q4t = qt(4);
Q5 = q(5);Q5t = qt(5);

M  = SX.sym('M',5,5);
C  = SX.sym('C',5,5);
H  = SX.sym('H',2,5);
Ht = SX.sym('Ht',2,5);
HQ  = SX.sym('HQ',2,1); %constraint matrix


matrices
define_functions


Minv     = finv5(M);
vel_dyn  = Minv* (u_gen - C*qt  -D*qt); %no grav
f        = -finv2( H*Minv*H' )  *   (H*vel_dyn + Ht*qt);
dynamics    = Minv* (u_gen - (C+D)*qt + (H')*f );


derivatives = [qt(1);qt(2);qt(3);qt(4);qt(5)];
yt = vertcat(derivatives,dynamics);

