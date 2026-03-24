function [decision_variables,...
    predicted_states_Nth_step,...
    desired_additional_thrust_and_body_rates] = MPC_fmincon_u(...
    prior_decision_variables,...
    weights,...
    x0,...
    v,...
    ssmodel,...
    lb,...
    ub,...
    H_MPC,...
    W_MPC)
    % TODO: MPC_fmincon only uses u as decision variables.

    % Define the objective function to minimize
    theta = [x0; v];
    cost_func_with_grad = @(U) getCostFunctionWithGradient(U, H_MPC, W_MPC, theta);
    
    initial_guess_U = prior_decision_variables;

    % set the inequal constraints
    A = [];
    b = [];

    % set the equal constraints
    Aeq = [];
    beq = [];

    % TODO
    nonlcon = [];

    % Call fmincon to perform the optimization with options
    % options = optimoptions('fmincon', 'SpecifyObjectiveGradient', true, 'SpecifyConstraintGradient', true, 'Display', 'iter', 'Algorithm', 'sqp', 'OutputFcn', @outfun);
    options = optimoptions('fmincon', 'SpecifyObjectiveGradient', true, 'Display', 'none', 'Algorithm', 'sqp');
    U_opt = fmincon(cost_func_with_grad, initial_guess_U, A, b, Aeq, beq, lb, ub, nonlcon, options);
  
    desired_additional_thrust_and_body_rates = [U_opt(1); U_opt(2); U_opt(3); U_opt(4)]; 
    predicted_states_Nth_step = getNthPredictedStates(U_opt, ssmodel.A, ssmodel.B);
    decision_variables = U_opt(1:size(prior_decision_variables,1),1);

end


function [f, gradf] = getCostFunctionWithGradient(U, H_MPC, W_MPC, theta) 
    
    f = 1/2 * U' * H_MPC * U + U' * W_MPC * theta;
    gradf = H_MPC * U + W_MPC * theta;

end 
