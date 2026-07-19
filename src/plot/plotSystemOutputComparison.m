function fhs = plotSystemOutputComparison(yA, yB, nameA, nameB)
    %% Plot outputs of a simulation or experiment
    %
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments
        yA    (1,1) struct
        yB    (1,1) struct
        nameA (1,1) string
        nameB (1,1) string
    end

    %% IMU Data
    fhs(1) = figure("NumberTitle", "off", "Name", "Comp. Outputs IMU");
    tiledlayout("TileSpacing", "tight");

    nIMUs = size(yA.IMUAcc, 2);

    compColorsIMU = [
        brighten(lines(3), 0.5);
        brighten(lines(3), -0.3);
        ];

    axisStrings = ["$x$"; "$y$"; "$z$"];

    for iIMU = 1:nIMUs
        nexttile;
        plot(yA.tout, squeeze(yA.IMUGyr(:,iIMU, :)), "-.");
        hold on;
        plot(yB.tout, squeeze(yB.IMUGyr(:,iIMU, :)), "-");
        title(sprintf("Angular Velocity IMU %d", iIMU));
        grid on;
        legend([nameA + " " + axisStrings; nameB + " " + axisStrings], "Interpreter", "latex");
        xlabel("time $t$ in s", "Interpreter", "latex");
        ylabel("angular velocity in rad/s", "Interpreter", "latex");
        colororder(compColorsIMU);

        nexttile;
        plot(yA.tout, squeeze(yA.IMUAcc(:,iIMU, :)), "-.");
        hold on;
        plot(yB.tout, squeeze(yB.IMUAcc(:,iIMU, :)), "-");
        title(sprintf("Acceleration IMU %d", iIMU));
        grid on;
        legend([nameA + " " + axisStrings; nameB + " " + axisStrings], "Interpreter", "latex");
        xlabel("time $t$ in s", "Interpreter", "latex");
        ylabel("acceleration in m/s$^2$", "Interpreter", "latex");
        colororder(compColorsIMU);
    end

    %% Tendon lengths

    compColorsL = [
        brighten(lines(size(yB.Lt,1)), 0.5);
        brighten(lines(size(yB.Lt,1)), -0.3);
        ];

    fhs(2) = figure("NumberTitle", "off", "Name", "Comp. Outputs Tendon Lengths");
    plot(yA.tout, yA.Lt, "-.");
    hold on;
    plot(yB.tout, yB.Lt, "-");
    title("Tendon lengths");
    grid on;
    xlabel("time $t$ in s", "Interpreter", "latex");
    ylabel("tendon length in m", "Interpreter", "latex");
    legend( ...
        [arrayfun(@(x) sprintf("%s tendon %d", nameA, x), 1:size(yA.Lt,1)), ...
        arrayfun(@(x) sprintf("%s tendon %d", nameB, x), 1:size(yA.Lt,1))]);
    colororder(compColorsL);

end
