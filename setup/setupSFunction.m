function setupSFunction(SimIn, model, copyHeaders)
    arguments
        SimIn struct
        model char
        copyHeaders logical = false
    end
    
    tol = 1e-9;
    scale = abs(SimIn.ScalingFactor);
    
    isGUAM = any(SimIn.vehID == [0 -1]);
    isHERO = (SimIn.vehID == 1);
    
    % Get s-function name and decide to compile
    if isGUAM && abs(scale - 1.0) < tol
        % sfuncName = 'LpC_wrapper_sfunc';
        sfuncName = 'LpC_wrapper_sfunc_new';
        compileNeeded = false;
    elseif isGUAM
        sfuncName = 'LpC_Scaled_wrapper_sfunc';
        compileNeeded = true;
    elseif isHERO
        sfuncName = 'LpC_Hero_wrapper_sfunc';
        outdir = fullfile(pwd,'vehicles','Lift+Cruise','obj');
        mexPath = fullfile(outdir, [sfuncName '.' mexext]);
        compileNeeded = ~isfile(mexPath);
    else
        sfuncName = sprintf('LpC_VEH%d_wrapper_sfunc', SimIn.vehID);
        outdir = fullfile(pwd,'vehicles','Lift+Cruise','obj');
        mexPath = fullfile(outdir, [sfuncName '.' mexext]);
        compileNeeded = ~isfile(mexPath);
    end
    
    if compileNeeded
        mex_LpC_sfunc(copyHeaders, sfuncName);
    end

    if ~bdIsLoaded(model), open(model); end
    
    % full path to the S-Function block:
    blk = [ model, '/', ...
      'Vehicle Simulation/', ...
      'Vehicle Model/', ...
      'Propulsion and Aerodynamic Forces and Moments/', ...
      'Propulsion and Aerodynamic Forces and Moments/S-Function/', ...
      'Lift+Cruise Forces//Moments' ];
    
    % switch it over
    set_param(blk, 'FunctionName', sfuncName);
end