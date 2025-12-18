function [w_co, f_co, sigma_total, sigma_co] = pilotCutOff(dx, dt)
    % Remove any DC bias so PSD reflects only the pilot’s activity
    x   = detrend(dx, 'constant');
    N = numel(x);                     % number of samples
    if N < 8
        w_co = 0; f_co = 0; sigma_total = 0; sigma_co = 0;   return
    end
    
    % Power spectral density
    fs  = 1/dt;                                % sample rate [Hz]
    winLen   = 2^floor(log2(min(N,1024)));     % e.g. 512, 256, ...
    winLen   = max(winLen,128);                % at least 128
    win      = hann(winLen,'periodic');        % Hann window
    
    noverlap = floor(0.5*winLen);              % 50 % overlap
    nFFT     = max( winLen , 256 );            % nFFT >= window length
    [PSD_Hz, f_Hz] = pwelch(x, win, noverlap, nFFT, fs, 'onesided');
    w_rad = 2*pi*f_Hz;                          % frequency vector [rad/s]
    PSD_rad = PSD_Hz / (2*pi);
    
    % Cumulative power
    cumPow = cumtrapz(w_rad, PSD_rad);
    totPow = cumPow(end);

    tiny = 1e-20;                   % numerical zero threshold
    if totPow < tiny
        w_co = 0;  f_co = 0;
        sigma_total = 0;  sigma_co = 0;
        return
    end
    
    % RMS
    sigma_total = sqrt(totPow / (2*pi));
    
    % Find w_co where cumulative power fraction = 0.5
    fracPow   = cumPow / totPow;                % normalised

    idx = find(fracPow >= 0.5, 1, 'first');
    if isempty(idx) || idx == 1         % numerical safety: handle plateau at 0
        w_co = w_rad(idx);
    else
        % linear interpolation between idx-1 and idx
        w1 = w_rad(idx-1);   w2 = w_rad(idx);
        f1 = fracPow(idx-1); f2 = fracPow(idx);
        w_co = w1 + (0.5 - f1) * (w2 - w1) / (f2 - f1);
    end

    f_co = w_co / (2*pi);                       % Hz
    
    % RMS up to w_co
    idx = w_rad <= w_co;
    sigma_co = sqrt(cumPow(find(idx, 1, 'last')) / (2*pi));
end