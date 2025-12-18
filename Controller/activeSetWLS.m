function [u, W, iter] = activeSetWLS(G, v, uMin, uMax, Wv, Wu, ud, gam)

%                            0 if u_i not saturated
% Working set syntax: W_i = -1 if u_i = uMin_i
%                           +1 if u_i = uMax_i

    if gam <= 0
        gam = 1e4;
    end
    iterMax = 50;

    n = length(uMin);

    u = (uMax + uMin)/2;
    W = zeros(n,1);

    gam_sq = sqrt(gam);

    A = [gam_sq*Wv*G ; Wu];
    b = [gam_sq*Wv*v ; Wu*ud];

    d = b - A*u; % Initial residual
    free = (W == 0); % Idx. of free variables (not saturated)

    for iter = 1:iterMax
        A_free = A(:,free); % Determine free vars
        p_free = lsqminnorm(A_free, d); % Solve the optimization problem for free vars

        % Zero all perturbations corresponding to active constraints and
        % insert perturbations into the free vars.
        p = zeros(n,1);
        p(free) = p_free;

        % Feasibility check
        u_new = u + p;
        not_feasible = (u_new < uMin) | (u_new > uMax);

        if ~any(not_feasible(free)) % It is feasible
            % Update
            u = u_new;
            d = d - A_free*p_free;

            lambda = W.*(A'*d); % Get Lagrangian multipliers

            if all(lambda >= 0)
                return;
            else
                % Remove the constraint associated with the most negative
                % lambda from the active set W.
                [~, idx_neg] = min(lambda);
                W(idx_neg) = 0;
                free(idx_neg) = 1;
            end

        else % It is not feasible

            % Compute the maximum factor alpha such that alpha*p is a
            % feasible perturbation. To do it, get the distances to the
            % boundaries. Alpha is between 0 and 1.
            dist_bound = ones(n,1);
            min_bound = free & p < 0;
            max_bound = free & p > 0;

            dist_bound(min_bound) = (uMin(min_bound) - u(min_bound)) ./ p(min_bound);
            dist_bound(max_bound) = (uMax(max_bound) - u(max_bound)) ./ p(max_bound);

            [alpha, idx_alpha] = min(dist_bound);

            % Update
            u = u + alpha*p;
            d = d - A_free*alpha*p_free;

            % Update the sign of the constraint
            W(idx_alpha) = sign(p(idx_alpha));
            free(idx_alpha) = 0;
        end
    end
end
