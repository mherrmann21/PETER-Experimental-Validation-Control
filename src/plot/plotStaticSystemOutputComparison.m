function fhs = plotStaticSystemOutputComparison(yA, yB, nameA, nameB, opts)
    %% Plot outputs of a (static) simulation or experiment
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

        opts.plotOverTension (1,1) logical = false;
        opts.setPointTensions (:,1) double = [];
    end

    nSetpoints = size(yA.Lt, 2);

    fhs = figure("NumberTitle", "off", "Name", "Comparison static outputs");
    tiledlayout("TileSpacing", "tight");

    if opts.plotOverTension && ~isempty(opts.setPointTensions)
        xVals = opts.setPointTensions;
        xString = "tendon tensions in N";
    else
        xVals = 1:nSetpoints;
        xString = "setpoint nr.";
    end

    %% Accelerations
    axisStrings = ["$x$"; "$y$"; "$z$"];

    compColorsIMU = [
        brighten(lines(3), 0.5);
        brighten(lines(3), -0.3);
        ];

    ax = nexttile;
    plot(xVals, squeeze(yA.Acc(:,1,:)), "-o");
    hold on;
    plot(xVals, squeeze(yB.Acc(:,1,:)), "--x");
    grid on;
    title("Accelerometer 1", "Interpreter", "latex");
    xlabel(xString, "Interpreter", "latex");
    ylabel("acceleration in m/s$^2$", "Interpreter", "latex");
    colororder(ax, compColorsIMU);
    legend([nameA + " " + axisStrings; nameB + " " + axisStrings], "Interpreter", "latex");

    ax = nexttile;
    plot(xVals, squeeze(yA.Acc(:,2,:)), "-o");
    hold on;
    plot(xVals, squeeze(yB.Acc(:,2,:)), "--x");
    grid on;
    title("Accelerometer 2", "Interpreter", "latex");
    xlabel(xString, "Interpreter", "latex");
    ylabel("acceleration in m/s$^2$", "Interpreter", "latex");
    colororder(ax, compColorsIMU);
    legend([nameA + " " + axisStrings; nameB + " " + axisStrings], "Interpreter", "latex");


    %% Tendon displacement
    compColorsL = [
        brighten(lines(size(yB.Lt,1)), 0.5);
        brighten(lines(size(yB.Lt,1)), -0.3);
        ];

    ax = nexttile;
    plot(xVals, yA.Lt, "-o");
    hold on;
    plot(xVals, yB.Lt, "--x");
    grid on;
    title("Tendon displacement", "Interpreter", "latex");
    xlabel(xString, "Interpreter", "latex");
    ylabel("tendon displacement in m", "Interpreter", "latex");
    colororder(ax, compColorsL);
    legend( ...
        [arrayfun(@(x) sprintf("%s tendon %d", nameA, x), 1:size(yA.Lt,1)), ...
        arrayfun(@(x) sprintf("%s tendon %d", nameB, x), 1:size(yA.Lt,1))]);

end
