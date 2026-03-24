

%% Set Crazyflie Vehicle Parameters

agent_size = 0.08; % unit: meter

% Gravitational constant:
g = 9.81;
g_vec = [0; 0; -g];

% The "true" mass of vehicle (unit:kg)
mass_true = 32e-3; % crazyflie 

% The "measured" mass of the vehicle (unit: kg)
mass_for_controller = mass_true * 1.00;

% The "true" mass moment of inertia of the vehicle (unit: kg m^2)
inertia_true = [...
        16.57 ,   0.83 ,   0.72 ;...
         0.83 ,  16.66 ,   1.80 ;...
         0.72 ,   1.80 ,  29.26  ...
    ] * 1e-6;

% Inverse of the moment of inertia
inertia_true_inverse = inv(inertia_true);

% The "measured" mass moment of inertia of the vehicle (unit: kg m^2).
inertia_for_controller = inertia_true;

% The the layout of the quadrotor vehicle:
% nrotor_vehicle_layout = [...
%       x_1 , x_2 , x_3 , x_4 ;...
%       y_1 , y_2 , y_3 , y_4 ;...
%       c_1 , c_2 , c_3 , c_4  ...
%   ];

% crazyflie "baseline" value for the x and y coordinates (unit: meter); 
% and for the "torque-to-thrust" ratio.
x_baseline = (0.092/2) / sqrt(2);
y_baseline = (0.092/2) / sqrt(2);
c_baseline = 0.00596;

% The "true" layout of vehicle
layout_true = [...
        [  1 , -1 , -1 ,  1 ] * x_baseline  ;...
        [ -1 , -1 ,  1 ,  1 ] * y_baseline  ;...
        [ -1 ,  1 , -1 ,  1 ] * c_baseline   ...
    ];

% The "measured" layout of vehicle
x_offset = x_baseline * 0.00; % unit: meter
y_offset = y_baseline * 0.00; % unit: meter
nrotor_vehicle_layout_for_controller = layout_true + ...
    repmat( [x_offset;y_offset;0.000] , 1 , size(layout_true,2) );

% Maximum and minimum thrust of propeller (Unit: newtons)
thrust_min = 0;
thrust_max = 0.59;

nrotor_vehicle_thrust_min = zeros(size(layout_true,2), 1);
nrotor_vehicle_thrust_max = thrust_max * ones(size(layout_true,2), 1);


%% Compute Equilibrium Thrust for each propeller of the vehicle

% Construct the "propeller-force-to-actuator" conversion matrix:
% nrotor_vehicle_M = [...
%         1 ,    1 ,    1 ,    1 ;...
%       y_1 ,  y_2 ,  y_3 ,  y_4 ;...
%      -x_1 , -x_2 , -x_3 , -x_4 ;...
%       c_1 ,  c_2 ,  c_3 ,  c_4  ...
%   ];
nrotor_vehicle_M = [...
        ones(1,size(nrotor_vehicle_layout_for_controller,2)) ;...
        nrotor_vehicle_layout_for_controller(2,:) ;...
       -nrotor_vehicle_layout_for_controller(1,:) ;...
        nrotor_vehicle_layout_for_controller(3,:)  ...
    ];

% nrotor_vehicle_M * [ thrust_1 ; ... ; thrust_N ] == [ mass*g ; 0 ; 0 ; 0 ]
equilibrium_thrust = nrotor_vehicle_M \ ...
    [mass_for_controller*g ; 0 ; 0 ; 0 ];


%% Load the PID parameters that are directly taken from crazyflie on-board firmware

pid_x_body_rate_kp = 250.0;
pid_x_body_rate_ki = 500.0;
pid_x_body_rate_kd = 2.5*0.1;
pid_x_body_rate_integrator_limit = 33.3;

pid_y_body_rate_kp = 250.0;
pid_y_body_rate_ki = 500.0;
pid_y_body_rate_kd =   2.5*0.1;
pid_y_body_rate_integrator_limit = 33.3;

pid_z_body_rate_kp = 120.0;
pid_z_body_rate_ki =  16.7;
pid_z_body_rate_kd =   0.0;
pid_z_body_rate_integrator_limit = 167.7;

% Specify the coefficients of the conversion from a uint16 binary command
% to the propeller thrust in Newtons
cmd_2_newtons_conversion_quadratic_coefficient  =  1.3385e-10;
cmd_2_newtons_conversion_linear_coefficient     =  6.4870e-6;

% Specify that max and min for the uint16 binary command
cmd_max = 2^16-1;
cmd_min = 0;
