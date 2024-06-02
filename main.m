%% GUIDE to use
%1. Start from mode = `pos_control` to verify this model works
%2. Go to mode = `TO_control`. -> Inequality is at 2.5e-1
%3. Go set same upper-lower constr in section 2: ubx_0,lbx_0 ->  Converges
%4. Comment out line 87 `ocp_model.set('constr_x0', [x0;Tmid])`  and set 
% diffent ubx_0,lbx_0 -> QP failure


if ~exist('simulink_opts')
    disp('using acados simulink default options')
    simulink_opts = get_acados_simulink_opts;
end
check_acados_requirements()


%% User options: 
%'pos_control' 
% 1. It sets the same upper and lower bound for Th
% 2. Has terminal constraint
% 3. Can have a different cost function, but not neccesary. 
% 4. Levenberg marquadt parameter set to 1e-2. 

mode = 'pos_control'; %Select from ['pos_control','TO_control'];

%% discretization-solvers-sim_method 
N    = 50;
Th   = N; %Horizon is in reality step length
Th_u = 0.35;
Th_l = 0.1;

Tmid = 0.5*(Th_u+Th_l);

dt_l = 0;
dt_u = 0;

if (strcmp(mode, 'pos_control'))
    %in classic pos control, Th is fixed
    Tmid = 0.35;
    Th_u = Tmid;
    Th_l = Th_u;
end

nlp_solver  = 'sqp';
qp_solver   = 'partial_condensing_hpipm'; %'partial_condensing_hpipm',
qp_solver_cond_N =  N; % for partial condensing
sim_method  = 'erk'; % integrator type.Also 'discrete'

%% Reference
% initial state:
q0 = zeros(5,1); 
w0 = zeros(5,1);
x0 = [q0;w0];

% reference:
% qref_ = [-1;1;0;1;0]; %only actuated
qref_ = DK(  [0;1.8;0;-0.7;0] );
qref = DK(qref_);
wref = [0;0;0;0;0];
xref = [qref;wref];


%% 0. Import model
model = leg_model_discrete(N);
nx = model.nx;
nu = model.nu;
ny = nu+nx;      % used in simulink example
ny_e = nx;
nst  = nx-1;     % number of true states

%% 1. State-input variables:
ocp_model = acados_ocp_model();
ocp_model.set('name', 'TO_position_MPC');

ocp_model.set('sym_x'   , model.sym_x   );
ocp_model.set('sym_u'   , model.sym_u   );
ocp_model.set('sym_xdot', model.sym_xdot);

%dynamics
if (strcmp(sim_method, 'erk'))
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.expr_f_expl);
elseif (strcmp(sim_method, 'discrete'))
    ocp_model.set('dyn_type', 'discrete');
    ocp_model.set('dyn_expr_phi', model.dyn_expr_phi)
end
%% 2. Constraints  - BOX:
ocp_model.set('constr_x0', [x0;Tmid]); % This line needs to exist to not
% fail. 

%1.  Input Box constraints
ocp_model.set('constr_lbu',[model.input_constraints(:,1)])
ocp_model.set('constr_ubu',[model.input_constraints(:,2)])
ocp_model.set('constr_Jbu',eye(nu))

%2.  State Box constraints
ocp_model.set('constr_lbx',[model.state_constraints(:,1);Th_l])
ocp_model.set('constr_ubx',[model.state_constraints(:,2);Th_u])
ocp_model.set('constr_Jbx',eye(nx))

ocp_model.set('constr_ubx_0',[x0;Th_l])
ocp_model.set('constr_lbx_0',[x0;Th_u])
ocp_model.set('constr_Jbx_0',eye(nx))

%A. slack state constraints to ensure feasibility 
% (Th not slacked):
% ocp_model.set('constr_Jsbx'  , eye(nx,nst));
% Z_x     = 200*eye(nst);
% z_x     = 1000*ones(nst,1);

%(Th slacked as well):
ocp_model.set('constr_Jsbx'  , eye(nx));
Z_x     = 200*eye(nx);
z_x     = 1000*ones(nx,1);

ocp_model.set('cost_Z',Z_x);
ocp_model.set('cost_z',z_x);

%3. Terminal cosntraint: only for pos control
if (strcmp(mode, 'pos_control'))
    ocp_model.set('constr_lbx_e',xref)
    ocp_model.set('constr_ubx_e',xref)
    ocp_model.set('constr_Jbx_e',eye(nst,nx))
    ocp_model.set('constr_Jsbx_e'  , eye(nst));
    Z_x_e     = 1e2*eye(nst);
    z_x_e     = 1e3*ones(nst,1);
    
    ocp_model.set('cost_Z_e',Z_x_e);
    ocp_model.set('cost_z_e',z_x_e);
end

%% 3. Cost
% non linear cost module:
ocp_model.set('cost_type_0', 'ext_cost');
ocp_model.set('cost_type'   ,'ext_cost')
ocp_model.set('cost_type_e'   ,'ext_cost')

%time optimal
Wt = 1e0;
cost_expr_ext_cost   = 0;
cost_expr_ext_cost_0 = Wt*model.sym_x(end);
cost_expr_ext_cost_e = 0;

%Position control Different cost:
% if (strcmp(mode, 'pos_control'))
%     Q = blkdiag(50* eye(5), 5* eye(5));
%     R = 10*eye(nu);
%     state = model.sym_x(1:10);
%     input = model.sym_u;
%     
%     state_penalty = (state-xref)' * Q* (state-xref);
%     input_penalty = input' *R*input; 
%     cost_expr_ext_cost   = (Th_u/N)* ( state_penalty + input_penalty);
%     cost_expr_ext_cost_e = state_penalty;
%     cost_expr_ext_cost_0 = cost_expr_ext_cost;
% end

ocp_model.set('cost_expr_ext_cost'    , cost_expr_ext_cost    );
ocp_model.set('cost_expr_ext_cost_0'  , cost_expr_ext_cost_0  );
ocp_model.set('cost_expr_ext_cost_e'  , cost_expr_ext_cost_e  );

%% 4. Options:
ocp_model.set('T', Th); %Time Horizon

ocp_opts = acados_ocp_opts();
ocp_opts.set('qp_solver_iter_max', 200); 
ocp_opts.set('nlp_solver_max_iter', 100);
ocp_opts.set('param_scheme_N', N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('levenberg_marquardt', 1e-2);

ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('ext_fun_compile_flags', ''); % '-O2'
ocp_opts.set('print_level', 3);
ocp_opts.set('sim_method_num_steps', 1); %Default 1
ocp_opts.set('nlp_solver_exact_hessian',1)
ocp_opts.set('exact_hess_cost',1)
ocp_opts.set('exact_hess_constr', 0);
ocp_opts.set('exact_hess_dyn', 0);


%% 5. Solve
ocp = acados_ocp(ocp_model, ocp_opts, simulink_opts);

%Initialization: 
ocp.set('init_x',lin_interpolate_traj([x0;Th_u],[xref;Th_u],N+1));
ocp.set('init_u',repmat([zeros(3,1)],   N, 1))

ocp.solve();
ocp.print('stat')
ocp.get('status')

%% 6. process results
utraj = ocp.get('u');
xtraj = ocp.get('x');
Th    = xtraj(end);
tmpc    = linspace(0,Th,N+1);

figure(Name='Position States')
tiledlayout(5,1)
for i=1:5
    nexttile
    plot(tmpc,xtraj(i,:),'b',LineWidth=2)
    hold on
    yline(xref(i),'g--')
    ylabel(['q_{',num2str(i),'}'],FontSize=15)
end

figure(Name='Control')
tiledlayout(3,1)
for i=1:3
    nexttile
    stairs(tmpc,[utraj(i,:),utraj(i,end)],'b',LineWidth=2)
    ylabel(['u_{',num2str(i),'}'],FontSize=15)
end
