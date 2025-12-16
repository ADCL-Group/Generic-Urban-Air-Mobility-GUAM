function [Out, vs] = setupTypes(SimIn, variantStruct)
  arguments
    SimIn struct
    variantStruct struct = []
  end

  % select scaling value
  if isfield(variantStruct, 'scaling')
    s = variantStruct.scaling;
    if ~(s > 0 || s == -1)
      error('Invalid scaling factor: must be –1 (default) or a positive number.');
    end
  else
    s = -1;   % flag to use default NASA model
  end
  SimIn.ScalingFactor = s;

  % select experiment
  if isfield(variantStruct,'expType')
    m = enumeration('ExperimentEnum');
    SimIn.expType = m(variantStruct.expType);
  else
    SimIn.expType = selectExperimentType;
  end

  % select vehicle model
  if isfield(variantStruct,'vehicleType')
    m = enumeration('VehicleEnum');
    SimIn.vehicleType = m(variantStruct.vehicleType);
  else
    SimIn.vehicleType = selectVehicleType;
  end

  % select atmosphere model
  if isfield(variantStruct,'atmosType')
    m = enumeration('AtmosphereEnum');
    SimIn.atmosType = m(variantStruct.atmosType);
  else
    SimIn.atmosType = selectAtmosphereType;
%    SimIn.atmosType = AtmosphereEnum.US_STD_ATMOS_76;
  end

  % select disable/enable turbulence model
  if isfield(variantStruct,'turbType')
    m = enumeration('TurbulenceEnum');
    SimIn.turbType = m(variantStruct.turbType);
  else
    SimIn.turbType = selectTurbulenceType;
%    SimIn.turbType = TurbulenceEnum.None;
  end

  % select flight mode
  %SimIn.mode = selectFlightMode;
  SimIn.mode = FlightModeEnum.CRUISE;

  % select controller model
  if isfield(variantStruct,'ctrlType')
    m = enumeration('CtrlEnum');
    SimIn.ctrlType = m(variantStruct.ctrlType);
  else
    SimIn.ctrlType = selectControllerType;
  end

  % select ref input
  if isfield(variantStruct,'refInputType')
    m = enumeration('RefInputEnum');
    SimIn.refInputType = m(variantStruct.refInputType);
  else
    SimIn.refInputType = selectRefInputType;
  end

  % select control surface actuator model
  if isfield(variantStruct,'actType')
    m = enumeration('ActuatorEnum');
    SimIn.actType = m(variantStruct.actType);
  else
    SimIn.actType = selectActuatorType(SimIn.ctrlType);
  end
  
  % select propulsion model
  if isfield(variantStruct,'propType')
    m = enumeration('PropulsionEnum');
    SimIn.propType = m(variantStruct.propType);
  else
    SimIn.propType = selectPropulsionType(SimIn.ctrlType);
  end
  
  % select aero/propulsion force/moment model
  if isfield(variantStruct,'fmType')
    m = enumeration('ForceMomentEnum');
    SimIn.fmType = m(variantStruct.fmType);
  else
    SimIn.fmType = selectForceMomentType;
  end
  
  % select EOM
  if isfield(variantStruct,'eomType')
    m = enumeration('EOMEnum');
    SimIn.eomType = m(variantStruct.eomType);
  else
    SimIn.eomType = selectEOMType;
  end
  
  % select sensor model
  if isfield(variantStruct,'sensorType')
    m = enumeration('SensorsEnum');
    SimIn.sensorType = m(variantStruct.sensorType);
  else
    SimIn.sensorType = selectSensorType;
  end

  % ADCL connection + ctrl mode
  % Defaults
  addr  = "127.0.0.1";
  pIn   = 5502;
  pOut  = 5501;

  % Pull from userStruct
  if isfield(variantStruct, 'connection')
    if isfield(variantStruct.connection,'address') && ~isempty(variantStruct.connection.address), addr = string(variantStruct.connection.address); end
    if isfield(variantStruct.connection,'portIn')  && ~isempty(variantStruct.connection.portIn),  pIn  = double(variantStruct.connection.portIn); end
    if isfield(variantStruct.connection,'portOut') && ~isempty(variantStruct.connection.portOut), pOut = double(variantStruct.connection.portOut); end
  end

  % Validate ports
  if ~(isfinite(pIn) && pIn==floor(pIn) && pIn>=1 && pIn<=65535),  pIn  = 5502; end
  if ~(isfinite(pOut)&& pOut==floor(pOut)&& pOut>=1&& pOut<=65535), pOut = 5501; end
  if pIn == pOut, error("portIn and portOut cannot be the same (%d).", pIn); end

  SimIn.simAddress    = char(addr);
  SimIn.simPortInput  = pIn;
  SimIn.simPortOutput = pOut;

  simIsRemote = ~(addr=="127.0.0.1" || addr=="localhost");

  if isfield(variantStruct,'ctrlMode')
    SimIn.ctrlMode = CtrlModeEnum(variantStruct.ctrlMode);
  else
      if simIsRemote
          SimIn.ctrlMode = CtrlModeEnum.RemotePilot; 
      else
          SimIn.ctrlMode = CtrlModeEnum.LocalPilot; 
      end
  end

  Out = SimIn;
  vs = variantStruct;
end
