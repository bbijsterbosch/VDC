% Dr. Barys Shyrokau
% Extended vehicle model for Group Project
% RO47017 Vehicle Dynamics & Control, 2024
% Use and distribution of this material outside the RO47017 course 
% only with the permission of the course coordinator

clc; clear all; close all;


% General
par.g = 9.81;
% Tire
par.Reff     = 0.3035;              % wheel effective radius, m
par.RL       = 0.294;               % wheel loaded radius, m
% Road
par.friction = 1.0;                 % friction coefficient, -
% Vehicle/Body (Camry)
par.mass     = 1380;                % vehicle mass, kg      
par.Ixx      = 649.1;               % body inertia around x-axis, kgm^2
par.Iyy      = 2415.6;              % body inertia around y-axis, kgm^2
par.Izz      = 2634.5;              % body inertia around z-axis, kgm^2
par.L        = 2.79;                % wheelbase, m
par.l_f      = 1.384;               % distance from front axle to CoG, m
par.hcg      = 0.609;               % height of vehicle CoG above road, m
par.hBf      = 0.788;               % half of front track width, m
par.hBr      = 0.782;               % half of rear track width, m
par.unm_f    = 71.5;                % front axle mass, kg       
par.unm_r    = 64.5;                % rear axle mass, kg    
par.hroll_f  = -0.05;               % front roll height, m             
par.hroll_r  = 0.12;                % rear roll height, m
par.hcg_r = par.hcg - par.hroll_f + (par.hroll_r - par.hroll_f)*par.l_f/par.L;
% Suspension
par.Kz_f     = 53690;               % front vertical stiffness, N/m     
par.Kz_r     = 64964;               % rear vertical stiffness, N/m  
par.Dz_f     = 2742;                % front vertical damping, Ns/m     
par.Dz_r     = 3018;                % rear vertical damping, Ns/m     
par.roll_gradient = 4.32 / 180*pi; %roll gradient deg/g
par.Kroll    = par.mass * par.g * par.hcg_r / par.roll_gradient; % total roll stiffness, N/rad    
par.Kroll_f  = 0.60*par.Kroll;          % front roll stiffness, N/rad    
par.Kroll_r  = par.Kroll - par.Kroll_f; % rear roll stiffness, N/rad  
par.Droll_f  = 4500;                % front roll damping, Ns/rad   
par.Droll_r  = 4500;                % rear roll damping, Ns/rad   
% Steering
par.i_steer  = 15.4;                % steering ratio, -
par.wn_ss    = 30;                  % natural frequency of steering system
par.zeta_ss = 0.7;                  % damping ratio of steering system
par.teta_ss  = 0.01;                % delay time of steering system                

% Additional
par.I        = diag([par.Ixx par.Iyy par.Izz]); % inertia matrix  
par.l_r      = par.L - par.l_f;                 % distance from rear axle to CoG, m
par.m_f      = par.mass * par.l_r / par.L;      % front sprung mass, kg
par.m_r      = par.mass * par.l_f / par.L;      % rear sprung mass, kg
par.length_f = par.m_f*par.g/par.Kz_f;          % front spring offset, m
par.length_r = par.m_r*par.g/par.Kz_r;          % rear spring offset, m

% Reference Generator (only for a fixed speed of 90 km/h)
par.Caf = 120000; % Front axle cornering stiffness
par.Car  = 190000; % Rear axle cornering stiffness
par.Kus = par.m_f/par.Caf - par.m_r/par.Car; % understeer gradient
% Dynamic response by the second-order transfer function parameters
par.wn      = 14.5;           % yaw frequency
par.kseta   = 0.6;            % yaw damping
par.tau     = 0.002;          % yaw delay

% Minimal speed
par.V0 = 50/3.6;                % initial speed, km/h
par.w0 = par.V0 / par.Reff;         % initial wheel angular velocity, rad/s

Ts = 0.01;
%% Add/ Change after this line
vx = load("vx.mat", "timeseries_vx");


% Maneuver settings
V_ref = vx.timeseries_vx;       
% pre-maneuver speed, km/h
% V_ref = 100/3.6;                % pre-maneuver speed, km/h

%% Acado setup
%% ACADO set up 
DifferentialState vx vy r yaw Xp Yp delta; % definition of controller states
Control d_delta; % definition of controller input
% controller model of the plant
beta = atan(par.l_r * tan (delta) / par.L);

f_ctrl = [
    dot(vx)  == 0;...
    dot(yaw) == vx / par.l_r * sin(beta);...
    dot(Xp)  == vx * cos(yaw + beta);...
    dot(Yp)  == vx * sin(yaw + beta)];

dynamic_ctrl = [
    dot(vx)     == vy*r;...
    dot(vy)     == -((par.Caf+par.Car)/(par.mass*vx))*vy + ((par.l_r*par.Car-par.l_f*par.Caf)/(par.mass*vx))*r + par.Caf*delta/par.mass;...
    dot(r)      == vy*(par.l_r*par.Car-par.l_f*par.Caf)/(par.Izz*vx) - r*((par.l_r^2)*par.Car+(par.l_f^2)*par.Caf)/(par.Izz*vx);...
    dot(yaw)    == r;...
    dot(Xp)     == vx*cos(yaw)- vy*sin(yaw);...
    dot(Yp)     == vx*sin(yaw) + vy*cos(yaw);...
    dot(delta)  == d_delta;
    ];


%% ACADO: controller formulation
acadoSet('problemname', 'PF_problem');
Np = 40;                                  % prediction horizon
ocp  = acado.OCP( 0.0, Np*Ts, Np);        % ACADO ocp

% Residual function definition based on ACADO
h = [diffStates ; controls];
hN = [diffStates];                       % terminal

% Initialization weights
W = acado.BMatrix(eye(length(h)));
WN = acado.BMatrix(eye(length(hN)));     % terminal

% Cost definition
ocp.minimizeLSQ(W,h);
ocp.minimizeLSQEndTerm(WN,hN);           % terminal

% Constraints definition
beta_thd       = 10 / 180*pi;            % absolute sideslip 
delta_thd   = 360/180*pi / par.i_steer;	 % absolute steering position
d_delta_thd = 800*pi/(180*par.i_steer);
% constraints in ACADO 
ocp.subjectTo( -delta_thd   <= delta    <= delta_thd);
ocp.subjectTo( -beta_thd   <= beta    <= beta_thd);
ocp.subjectTo( -d_delta_thd <= d_delta <= delta_thd);
% define ACADO prediction model
ocp.setModel(dynamic_ctrl);
    
% ACADO settings [Don't change these settings in your HMA]
mpc = acado.OCPexport( ocp );
mpc.set('HESSIAN_APPROXIMATION', 'GAUSS_NEWTON');       % solving algorithm
mpc.set('DISCRETIZATION_TYPE', 'MULTIPLE_SHOOTING');    % discretization algorithm
mpc.set('INTEGRATOR_TYPE', 'INT_IRK_GL2');              % intergation algorithm
mpc.set('NUM_INTEGRATOR_STEPS', 3*Np);                  % number of integration steps
mpc.set('LEVENBERG_MARQUARDT', 1e-4);                   % value for Levenberg-Marquardt regularization -> affects performance
mpc.set('SPARSE_QP_SOLUTION', 'FULL_CONDENSING_N2');
mpc.set('QP_SOLVER', 'QP_QPOASES3');
mpc.set('MAX_NUM_QP_ITERATIONS', 20) ;
mpc.set('HOTSTART_QP','YES');
mpc.set('GENERATE_SIMULINK_INTERFACE', 'YES');

%% Export and Compile flags
EXPORT  = 1;
COMPILE = 1;

% export code to the defined folder
if EXPORT
    mpc.exportCode('export_MPC');
end

% compilation of the S-function using autogenerated make script
if COMPILE
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_MPC/qpoases3')
    cd export_MPC
    make_acado_solver_sfunction
    copyfile('acado_solver_sfun.mex*', '../')
    cd ..
end

%% initial MPC settings
disp('Initialization')
X0       = [60/3.6 0 0 0 0 0 0];             % initial state conditions
% initialize controller bus
input.x  = repmat(X0, Np + 1, 1).';      % size Np + 1
input.od = zeros(Np + 1, 1);            % size Np + 1
Uref     = zeros(Np, 1);
input.u  = Uref.';
input.y  = [repmat(X0, Np, 1) Uref].';   % reference trajectory, size Np + 1
input.yN = X0.';                        % terminal reference, size Np + 1
% redefined in Simulink
input.W  = diag([0 0 0 0 0 5e-3 1e-1 0]);     % weight tuning !! Tune them in the Simulink model !!
input.WN = diag([0 0 0 0 0 0 0]);             % terminal weight tuning
input.x0 = X0.';
% controller bus initialization
init.x   = input.x(:).';                  % state trajectory
init.u   = input.u(:).';                  % control trajectory
init.y   = input.y(:).';                  % reference trajectory (up to Np - 1)
init.yN  = input.yN(:).';                % terminal reference value (only for Np)
init.W   = input.W(:).';                  % stage cost matrix (up to Np - 1)
init.WN  = input.WN(:).';                % terminal cost matrix (only for Np)
init.x0  = input.x0(:).';
