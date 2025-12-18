function saveFlightSummary(SimOut, SimPar, SimIn, PilotInputs, FileName)
    if nargin < 4, PilotInputs   = []; end
    if nargin < 5, FileName = ''; end

    IC = SimIn.IC;
    Act = SimIn.Act;
    Eng = SimIn.Eng;
    Units = SimIn.Units;

    Fail = [];
    if isstruct(SimPar) && isfield(SimPar, 'Value') && ...
            isstruct(SimPar.Value) && isfield(SimPar.Value, 'Fail')
        Fail = SimPar.Value.Fail;
    end

    PathPlan = [];
    if isstruct(SimIn) && isfield(SimIn, 'PathPlan')
        PathPlan = SimIn.PathPlan;
    end

    folder = 'FlightData';
    prefix = 'flight_summary';

    if ~exist(folder, 'dir')
        mkdir(folder);
    end

    if ~isempty(FileName)
        % If user passed only a name, not a full path, put it in the folder
        [p, name, ext] = fileparts(FileName);
        if isempty(p)
            p = folder;   % save inside FlightData
        end
        if isempty(ext)
            ext = '.mat'; % ensure extension
        end
        filename = fullfile(p, [name ext]);
    else
        files = dir(fullfile(folder, [prefix '*.mat']));
        nextNum = numel(files) + 1;
        filename = fullfile(folder, sprintf('%s%d.mat', prefix, nextNum));
    
        if exist(filename, 'file')
            stamp = char(datetime('now','Format','yyyyMMdd_HHmmssSSS'));
            filename = fullfile(folder, sprintf('%s_%s.mat', prefix, stamp));
        end
    end

    save(filename, 'SimOut', 'Fail', 'PilotInputs', 'PathPlan', ...
            'IC', 'Act', 'Eng', 'Units', '-v7.3');

    fprintf('Saved summary to %s\n', filename);
end
