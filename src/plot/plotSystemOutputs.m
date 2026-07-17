function fhs = plotSystemOutputs(yData, Name)
    %% Plot outputs of a simulation or experiment
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        yData   (1,1) struct
        Name    (1,1) string
    end

    %% IMU Data
    fhs = figure("NumberTitle", "off", "Name", Name + ": Outputs IMU");
    tiledlayout("TileSpacing", "tight");

    nIMUs = size(yData.IMUAcc, 2);

    for iIMU = 1:nIMUs
        nexttile;
        plot(yData.tout, squeeze(yData.IMUGyr(:,iIMU, :)));
        title(sprintf("Angular Velocity IMU %d", iIMU), "Interpreter", "latex");
        grid on;
        legend("$x$", "$y$", "$z$", "Interpreter", "latex");
        xlabel("time $t$ in s", "Interpreter", "latex");
        ylabel("angular velocity in rad/s", "Interpreter", "latex");

        nexttile;
        plot(yData.tout, squeeze(yData.IMUAcc(:,iIMU, :)));
        title(sprintf("Acceleration IMU %d", iIMU), "Interpreter", "latex");
        grid on;
        legend("$x$", "$y$", "$z$", "Interpreter", "latex");
        xlabel("time $t$ in s", "Interpreter", "latex");
        ylabel("acceleration in m/s$^2$", "Interpreter", "latex");
    end

    %% FFT IMU Data

    % Get sampling frequency
    Fs = 1/mean(diff(yData.tout));

    fhs(end+1) = figure("NumberTitle", "off", "Name", Name + ": Outputs FFT IMU");
    tiledlayout("TileSpacing", "tight");

    nIMUs = size(yData.IMUAcc, 2);

    for iIMU = 1:nIMUs

        % Get single-sided amplitude spectrum of the signal
        [P1Gyr, f] = fftSSAS(squeeze(yData.IMUGyr(:,iIMU,:)).', Fs);
        [P1Acc, ~] = fftSSAS(squeeze(yData.IMUAcc(:,iIMU,:)).', Fs);

        nexttile;
        semilogy(f, P1Gyr);
        title(sprintf("Angular Velocity IMU %d", iIMU), "Interpreter", "latex");
        grid on;
        legend("$x$", "$y$", "$z$", "Interpreter", "latex");
        xlabel("frquency $f$ in Hz", "Interpreter", "latex");
        ylabel("signal amplitude", "Interpreter", "latex");
        xlim([0, 200]);

        nexttile;
        semilogy(f, P1Acc);
        title(sprintf("Acceleration IMU %d", iIMU), "Interpreter", "latex");
        grid on;
        legend("$x$", "$y$", "$z$", "Interpreter", "latex");
        xlabel("frquency $f$ in Hz", "Interpreter", "latex");
        ylabel("signal amplitude", "Interpreter", "latex");
        xlim([0, 200]);
    end

    %% Tendon length

    fhs(end+1) = figure("NumberTitle", "off", "Name", Name + ": Outputs Cable Lengths");
    plot(yData.tout, yData.Lc);
    title("Cable lengths");
    grid on;
    xlabel("time $t$ in s", "Interpreter", "latex");
    ylabel("cable length in m", "Interpreter", "latex");
    legend(arrayfun(@(x) sprintf("Cable %d", x), 1:size(yData.Lc,1)));
end