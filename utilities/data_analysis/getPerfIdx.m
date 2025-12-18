function PI = getPerfIdx(T_tilde, C_tilde, weights)
    % Computes the final performance index
    %
    % Inputs:
    %   T_tilde - tracking index (scalar)
    %   C_tilde - control effort index (scalar)
    %   weights - [1x2] vector [w_T, w_C]
    %
    % Output:
    %   PI - final performance index (scalar)

    % Validate weights
    assert(length(weights) == 2, 'weights must be [w_T, w_C]');
    assert(abs(sum(weights) - 1) < 1e-6, 'Weights must sum to 1');

    % Compute performance index
    PI = 1 - (weights(1) * T_tilde + weights(2) * C_tilde);
end
