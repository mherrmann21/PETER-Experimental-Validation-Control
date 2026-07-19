function y = getSystemOutputsFromExpData(expData, opts)
    %% Get system output struct from raw experiment data
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        expData (1,1) struct

        % Output time vector
        % If specified, the data is re-sampled at these time values
        opts.tVecOutput     (:,1) double = nan;

        % Time shift for the outputs (shifts the data w.r.t. the original
        % time vector)
        opts.tShift         (:,1) double = 0;

        % Filter data with low-pass?
        opts.filterData     (1,1) logical = false;
        opts.LPCutOffFrequ  (1,1) double  = 1e3;
    end

    %% Get raw data from struct
    yIMUAcc1 = squeeze(expData.IMUData.sensor_1_acc.Data);
    yIMUGyr1 = squeeze(expData.IMUData.sensor_1_gyro.Data);
    yIMUAcc2 = squeeze(expData.IMUData.sensor_2_acc.Data);
    yIMUGyr2 = squeeze(expData.IMUData.sensor_2_gyro.Data);
    yLt      = expData.tendonDisplacementActual_m.Data;
    tout     = expData.tendonDisplacementActual_m.Time - expData.tendonDisplacementActual_m.Time(1) + opts.tShift;


    %% Filter data
    if opts.filterData
        % Get data sample time
        hData = mean(diff(tout));

        fs = 1/hData;
        fc = opts.LPCutOffFrequ; % low-pass cutoff (Hz)
        [b,a] = butter(4, fc/(fs/2), 'low');

        yIMUAcc1 = filtfilt(b, a, yIMUAcc1.').';
        yIMUGyr1 = filtfilt(b, a, yIMUGyr1.').';
        yIMUAcc2 = filtfilt(b, a, yIMUAcc2.').';
        yIMUGyr2 = filtfilt(b, a, yIMUGyr2.').';
        yLt      = filtfilt(b, a, yLt);
    end


    %% Sample data
    if ~isnan(opts.tVecOutput)
        yIMUAcc1 = interp1(tout, yIMUAcc1.', opts.tVecOutput).';
        yIMUGyr1 = interp1(tout, yIMUGyr1.', opts.tVecOutput).';
        yIMUAcc2 = interp1(tout, yIMUAcc2.', opts.tVecOutput).';
        yIMUGyr2 = interp1(tout, yIMUGyr2.', opts.tVecOutput).';
        yLt      = interp1(tout, yLt     , opts.tVecOutput).';
        tout = opts.tVecOutput;
    end

    %% Assign to output struct
    y = struct();
    y.yAll = [];
    y.IMUAcc = cat(2, reshape(yIMUAcc1, 3, 1, []), reshape(yIMUAcc2, 3, 1, []));
    y.IMUGyr = cat(2, reshape(yIMUGyr1, 3, 1, []), reshape(yIMUGyr2, 3, 1, []));
    y.Lt     = yLt;
    y.tout   = tout;
end
