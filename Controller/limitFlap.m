function u_limits   = limitFlap(u_curr, PosLimits, RateLimits)
% limitFlap returns the minimum and maximum feasible flap deflection (fMin,fMax)
% for a given aileron deflection a, so that both (f+a) and (f-a) lie within ±limit.
    % a = u_curr(11);
    % 
    % % From -limit <= f+a <= limit:
    % fMin1 = PosLimits(2,10) - a;   % lower bound
    % fMax1 = PosLimits(1,10) - a;   % upper bound
    % 
    % % From -limit <= f-a <= limit:
    % fMin2 = PosLimits(2,11) + a;   % lower bound
    % fMax2 = PosLimits(1,11) + a;   % upper bound
    % 
    % % Combine the bounds
    % fMin = max(fMin1, fMin2);
    % fMax = min(fMax1, fMax2);
    % 
    % u_limits = [PosLimits(:,1:13); RateLimits(1:13)'];
    % u_limits(1,10) = fMax;
    % u_limits(2,10) = fMin;

    f = u_curr(10);

    % From -limit <= f+a <= limit:
    aMin1 = PosLimits(2,10) - f;   % lower bound
    aMax1 = PosLimits(1,10) - f;   % upper bound
    
    % From -limit <= f-a <= limit:
    aMin2 = f - PosLimits(1,11);   % lower bound
    aMax2 = f - PosLimits(2,11);   % upper bound
    
    % Combine the bounds
    aMin = max(aMin1, aMin2);
    aMax = min(aMax1, aMax2);

    u_limits = [PosLimits(:,1:13); RateLimits(1:13)'];
    u_limits(1,11) = aMax;
    u_limits(2,11) = aMin;
end
