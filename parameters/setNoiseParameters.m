
%% SPECIFY THE MEASUREMENT NOISE PARAMETERS

% Degrees to radians conversion
deg2rad = pi/180;

% Create a random number gerneator that will be used to generate seeds
measurement_noise_randStream_seed = 42;
measurement_noise_randStream_for_seeds = RandStream.create(...
        'mrg32k3a' ,...
        'NumStreams' , 1 ,...
        'Seed' , measurement_noise_randStream_seed ...        
    );

% Noise For the poisition meaurement (units of meters for the mean)
measurement_noise_p_mean    =  zeros(3,1);
measurement_noise_p_stddev  = 1 * [ 0.1 ; 0.1 ; 0.1  ];
measurement_noise_p_var     =  measurement_noise_p_stddev.^2;
measurement_noise_p_seed    =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);

% Noise For the translation velocity meaurement
measurement_noise_p_dot_mean    =  zeros(3,1);
measurement_noise_p_dot_stddev  =  2 * measurement_noise_p_stddev;
measurement_noise_p_dot_var     =  measurement_noise_p_dot_stddev.^2;
measurement_noise_p_dot_seed    =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);

% Noise For the euler anlge meaurement
measurement_noise_psi_mean    =  zeros(3,1);
measurement_noise_psi_stddev  =  2 * [ 1 ; 1 ; 1 ] * deg2rad;
measurement_noise_psi_var     =  measurement_noise_psi_stddev.^2;
measurement_noise_psi_seed    =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);

% Noise For the euler anlgar velocity meaurement
measurement_noise_psi_dot_mean    =  zeros(3,1);
measurement_noise_psi_dot_stddev  =  2 * measurement_noise_psi_stddev;
measurement_noise_psi_dot_var     =  measurement_noise_psi_dot_stddev.^2;
measurement_noise_psi_dot_seed    =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);

% Noise For the full state measurement stacked together
measurement_noise_full_state_mean = [...
        measurement_noise_p_mean        ;...
        measurement_noise_p_dot_mean    ;...
        measurement_noise_psi_mean      ;...
        measurement_noise_psi_dot_mean   ...
    ];
measurement_noise_full_state_var = [...
        measurement_noise_p_var        ;...
        measurement_noise_p_dot_var    ;...
        measurement_noise_psi_var      ;...
        measurement_noise_psi_dot_var   ...
    ];

measurement_noise_full_state_seed = [...
        measurement_noise_p_seed        ;...
        measurement_noise_p_dot_seed    ;...
        measurement_noise_psi_seed      ;...
        measurement_noise_psi_dot_seed   ...
    ];

% The full state covariance matrix
%   - Specify a diagonal matrix of the variances as the baseline
measurement_noise_full_state_covariance_matrix = diag( measurement_noise_full_state_var );
%   - Add some small covariance between the x and y position measurements
% measurement_noise_full_state_covariance_matrix([1,2],[2,1]) = 0.00;
%   - Compute the decomposition needed for computing a multi-variate
%     Gaussian sample given a sample from a standard normal distribution
%     The full() function converts a sparse matrix to a full matrix. 
%     U will contain the eigenvectors stored as columns;
%     D will be a diagonal matrix with the c orresponding eigenvalues
[U , D] = eig( full( measurement_noise_full_state_covariance_matrix ) ); 
measurement_noise_full_state_covariance_decomposition = U * sqrt(D);


% Noise For the gyroscope meaurement of the body rates
measurement_noise_gyroscope_mean    =  zeros(3,1);
measurement_noise_gyroscope_stddev  =  [ 1.0 ; 1.0 ; 1.0  ] * deg2rad;
measurement_noise_gyroscope_var     =  measurement_noise_gyroscope_stddev.^2;
measurement_noise_gyroscope_seed    =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);
% The gyroscope covariance matrix
%   - Specify a diagonal matrix of the variances as the baseline
measurement_noise_gyroscope_covariance_matrix = diag( measurement_noise_gyroscope_var );
%   - Compute the decomposition needed for computing a multi-variate
%     Gaussian sample given a sample from a standard normal distribution
[U , D] = eig( full( measurement_noise_gyroscope_covariance_matrix ) );
measurement_noise_gyroscope_covariance_decomposition = U * sqrt(D);

% Noise For the accelerometer meaurement of the body frame accelerations
measurement_noise_accelerometer_mean    =  zeros(3,1);
measurement_noise_accelerometer_stddev  =  [ 1.0 ; 1.0 ; 1.0  ];
measurement_noise_accelerometer_var     =  measurement_noise_accelerometer_stddev.^2;
measurement_noise_accelerometer_seed    =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);
% The accelerometer covariance matrix
%   - Specify a diagonal matrix of the variances as the baseline
measurement_noise_accelerometer_covariance_matrix = diag( measurement_noise_accelerometer_var );
%   - Compute the decomposition needed for computing a multi-variate
%     Gaussian sample given a sample from a standard normal distribution
[U , D] = eig( full( measurement_noise_accelerometer_covariance_matrix ) );
measurement_noise_accelerometer_covariance_decomposition = U * sqrt(D);
