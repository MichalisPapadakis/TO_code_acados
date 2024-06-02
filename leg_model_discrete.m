function model = leg_model_discrete(N)
%leg_model_discrete: This function creates the struct that contains the 
% dynamic and system information for the leg.


import casadi.*
%% 1.  system dimensions:
nx = 11; 
nu = 3; 

%% 2.  system parameters:
%1. Damping 
D = 2.5e-3*ones(5,1); %good value

%% 3.  symbolic variables:
%state:
q  = SX.sym('q' ,5) ; 
qt = SX.sym('qt',5) ; 
Th = SX.sym('Th',1) ;    %horizon time as state


Q1 = q(1);Q1t = qt(1);
Q2 = q(2);Q2t = qt(2);
Q3 = q(3);Q3t = qt(3);
Q4 = q(4);Q4t = qt(4);
Q5 = q(5);Q5t = qt(5);

sym_x    = vertcat(q, qt,Th);
sym_xdot = SX.sym('xdot', nx, 1);

%input: 
% dth  = SX.sym('dth',1) ;            % Dth as optimization variable - pseudo variable that is constr 0

tau = SX.sym('tau',nu,1);         % horizontal force acting on cart [N]
sym_u = tau;

%% 4.a dynamics helpers: 
M  = SX.sym('M',5,5);
C  = SX.sym('C',5,5);
% G  = SX.sym('G',5,1);
H  = SX.sym('H',2,5);
Ht = SX.sym('Ht',2,5);

HQ  = SX.sym('HQ',2,1); %constraint matrix

%fill matrices and define helper functions
matrices
define_functions %finv5,finv2
%% 4.b. dynamics: 
u_gen = [tau(1);tau(2);0;tau(3);0];
D = diag(D);


Minv = finv5(M);
vel_dyn  = Minv* (u_gen - C*qt  -D*qt); %no grav

f=  -finv2( H*Minv*H' )  *   (H*vel_dyn + Ht*qt);

dynamics    = Minv* (u_gen - (C+D)*qt + (H')*f );
%% 4.b Explicit dynamics:
derivatives = [qt(1);qt(2);qt(3);qt(4);qt(5)];
Th_dynamics = 0; %dth will be constrainted to 0. This is so jacobian not singular


dt = Th/N;
xstate = sym_x(1:nx-1);

% expr_f_impl = sym_xdot - expr_f_expl;
% expr_f_disc  = sym_x + expr_f_expl*dt;
% rk4:
k1 = leg_dynamics_f(xstate            ,u_gen,D);
k2 = leg_dynamics_f(xstate + dt/2*k1  ,u_gen,D);
k3 = leg_dynamics_f(xstate + dt/2*k2  ,u_gen,D);
k4 = leg_dynamics_f(xstate + dt  *k3  ,u_gen,D);

expr_f_expl = vertcat(derivatives*dt ,dynamics*dt,Th_dynamics);
expr_f_disc  = [ xstate + dt/6*(k1 +2*k2 +2*k3 +k4);Th+ Th_dynamics];

%% 4.c Implicit dynamics (not working):

% temp = inv(H*Minv*H');
% Nc = eye(5) - Minv*(H')*temp*H ; % constraint null-space matrix
% 
% 
% expr_f_impl = [sym_xdot(1:5)-qt;(Nc')*(M*sym_xdot(6:10) + (C+D)*qt +  -u_gen)];

%% 5.  system constraints:
%1. state constraints
state_constraints = zeros(10,2);

define_state_constraints

%velocity constraints
max_vel = 10; % [rad/s] from olympian_rl
state_constraints(6:10,:) = max_vel*ones(5,2)*[-1,0;0,1];


%2. input constraints
input_constraints = [10;10;10]*[-1,1];

%3. kinematic loop closure constraint
path_constraints = HQ;

%workspace constraints:



%% 6.  populate structure:
model.name = 'fr_leg_disc';

model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
% model.expr_f_impl = expr_f_impl;
model.dyn_expr_phi = expr_f_disc;
model.input_constraints = input_constraints;
model.state_constraints = state_constraints;
model.path_constraints = path_constraints;
end