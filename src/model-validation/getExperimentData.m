function [tensionsDS, yExp] = getExperimentData(fileName, tout, opts)
    %% Get all required data from experiment recordings in the correct format
    arguments (Input)
        fileName

        % Time vector for the output signals (tensions and system outputs)
        tout

        % Start and end time, to which the experiment data is cropped
        %  (measured relative to beginning of the data, not the absolute
        %  time values in the experimental data)
        opts.tStartOffset (1,1) double = 0;

        % Vector with indices of the tendons that should be used in the
        % output data; can be used to exclude tendons
        opts.usedTendons  (:,1) double = [1,2,3,4];

        % Time to shift the output data w.r.t. the original time
        opts.tYShift      (1,1) double = 0;

        % Wether to use the measured tensions (false) or the reference
        % values (true) as the tensions
        opts.useReferenceTensions (1,1) logical = false;

        % Cut-off frequency for the tendon tension low-pass filtering
        opts.fCTendons            (1,1) double = 100;

        % Cut-off frequency for the output signals low-pass filtering
        opts.fCOutput             (1,1) double = 500;

        opts.plotFFT              (1,1) logical = false;
    end

    %% Constants/Parameters

    % Spool inertia
    % Pasted from identification
    JSpool = [
        0.00157588014020007
        0.00147053481788324
        0.00150632183204949
        0.00118615475862391
        ]*10;

    % Spool radius
    rS = 90/2*1e-3;


    %% Load data

    disp("Preparing experiment data...");

    expDataTS = load(fileName);

    % Plot raw data
    plotExpData(expDataTS);

    toutData = expDataTS.tendonTensionActual_N.Time - expDataTS.tendonTensionActual_N.Time(1);

    % Get sample times
    h     = mean(diff(tout));
    hData = mean(diff(toutData));

    % Time vector with given offset; used to get the corresponding time
    % segment from data
    toutOffset = tout + opts.tStartOffset;


    %% Prepare tendon tensions
    % considering the additional spool inertia in the measurements

    if opts.useReferenceTensions
        outputTorqueActual = double(expDataTS.DriveTargetValues.TargetOutputTorque.Data) / 1000;
    else
        outputTorqueActual = double(expDataTS.DriveActualValues.OutputTorqueActualValues.Data) / 1000;
    end
    vel_mRPM = double(expDataTS.DriveActualValues.VelocityActualValues.Data);
    vel_rad = vel_mRPM / 60 / 1000 * 2 * pi;

    % Filter velocity
    fs = 1/hData;
    fcV = 25;
    [bV,aV] = butter(4, fcV/(fs/2));
    vel_rad_filt = filtfilt(bV, aV, vel_rad);

    % Acceleration
    acc_rad = gradient((vel_rad_filt).', h).';

    % Inertia torque
    torque_spool = acc_rad.' .* JSpool;

    % Filter torque
    fcT = opts.fCTendons;%250; % low-pass cutoff (Hz)
    [bT,aT] = butter(4, fcT/(fs/2));
    outputTorqueActualFilt = filtfilt(bT, aT, outputTorqueActual);
    torque_spoolFilt = filtfilt(bT, aT, torque_spool.');

    % Output torque with subtracted inertia torque
    outputTorqueTension = outputTorqueActualFilt + abs(torque_spoolFilt);


    % Tensions
    tensions = - outputTorqueTension ./ rS;

    % Downsample data
    tensionsDS = interp1(toutData, tensions(:,opts.usedTendons), toutOffset);


    %% Plot tensions
    figure("Name", "Simulation Input data", "NumberTitle", "Off");
    tiledlayout;
    for iT = opts.usedTendons.'
        nexttile;
        plot(toutData, expDataTS.tendonTensionActual_N.Data(:,iT), "DisplayName", "Original");
        hold on;
        plot(toutData, tensions(:,iT), "DisplayName", "Preprocessed");
        grid on;
        xlabel("time in s", "Interpreter", "latex");
        ylabel("tendon tension in N", "Interpreter", "latex");
        title(sprintf("Measured and Preprocessed Data Tendon %d", iT));
        legend("Interpreter", "latex");
    end

    %% Plot tension FFTs (raw data)

    if opts.plotFFT
        % Get sampling frequency
        Fs = 1/mean(diff(expDataTS.tendonTensionActual_N.Time));
        % Get single-sided amplitude spectrum of the signals
        [P1T, f] = fftSSAS(expDataTS.tendonTensionActual_N.Data, Fs);

        figure("Name", "Raw data Tendon Tensions FFT", "NumberTitle", "Off");
        semilogy(f, P1T);
        grid on;
        xlabel("frquency $f$ in Hz", "Interpreter", "latex");
        ylabel("signal amplitude", "Interpreter", "latex");
        title("Comparison Measured and Preprocessed Data");
        legend(arrayfun(@(x) sprintf("cable %d", x), 1:4),"Interpreter", "latex");
    end

    %% Plot IMU FFTs (with raw data)

    if opts.plotFFT
        % Get sampling frequency
        Fs = 1/mean(diff(expDataTS.IMUData.sensor_1_acc.Time));

        figure("NumberTitle", "off", "Name", "Raw data IMU FFT");
        tiledlayout("TileSpacing", "tight");

        nIMUs = 2;

        % Get single-sided amplitude spectrum of the signals
        [~, f] = fftSSAS(squeeze(expDataTS.IMUData.sensor_1_acc.Data).', Fs);
        P1Acc = {
            fftSSAS(squeeze(expDataTS.IMUData.sensor_1_acc.Data).', Fs);
            fftSSAS(squeeze(expDataTS.IMUData.sensor_2_acc.Data).', Fs);
            };
        P1Gyr = {
            fftSSAS(squeeze(expDataTS.IMUData.sensor_1_gyro.Data).', Fs);
            fftSSAS(squeeze(expDataTS.IMUData.sensor_2_gyro.Data).', Fs);
            };

        % Plot data
        for iIMU = 1:nIMUs

            nexttile;
            semilogy(f, P1Gyr{iIMU});
            title(sprintf("Angular Velocity IMU %d", iIMU), "Interpreter", "latex");
            grid on;
            legend("$x$", "$y$", "$z$", "Interpreter", "latex");
            xlabel("frquency $f$ in Hz", "Interpreter", "latex");
            ylabel("signal amplitude", "Interpreter", "latex");
            %xlim([0, 200]);

            nexttile;
            semilogy(f, P1Acc{iIMU});
            title(sprintf("Acceleration IMU %d", iIMU), "Interpreter", "latex");
            grid on;
            legend("$x$", "$y$", "$z$", "Interpreter", "latex");
            xlabel("frquency $f$ in Hz", "Interpreter", "latex");
            ylabel("signal amplitude", "Interpreter", "latex");
            %xlim([0, 200]);
        end
    end

    %% Prepare outputs
    % Get output data; filter and resample/crop data
    % Note: Filter lowpass frequency is chosen according to simulation sampling
    % frequ.

    % Remove unused tendons
    expDataTS.tendonDisplacementActual_m.Data = expDataTS.tendonDisplacementActual_m.Data(:,opts.usedTendons);

    yExp = getSystemOutputsFromExpData(expDataTS, ...
        "filterData", true, "LPCutOffFrequ", opts.fCOutput, ...
        "tVecOutput", toutOffset, ...
        "tShift", opts.tYShift...
        );

    % Assign output vector without offset
    yExp.tout = tout;

end