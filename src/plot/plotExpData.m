function fhs = plotExpData(expData)
    %% Plot raw data from experiments
    arguments
        expData (1,1) struct
    end

    %% Tendon Data

    fhs(1) = figure("Name", "Tendon Data", "NumberTitle", "off");
    tiledlayout("vertical");
    nexttile;
    plot(expData.tendonDisplacementActual_m);
    grid on;
    axis tight;
    legend(arrayfun(@(x)sprintf("Tendon %d", x), 1:4));

    nexttile;
    plot(expData.tendonTensionActual_N);
    hold on;
    plot(expData.tendonTensionTarget_N);
    grid on;
    axis tight;
    legend([ ...
        arrayfun(@(x)sprintf("Tendon %d Actual", x), 1:4),...
        arrayfun(@(x)sprintf("Tendon %d Target", x), 1:4)]);

    %% IMU Data

    fhs(2) = figure("Name", "IMU Data", "NumberTitle", "off");
    tiledlayout(2,2);

    IMUPlotData = {
        expData.IMUData.sensor_1_gyro;
        expData.IMUData.sensor_1_acc;
        expData.IMUData.sensor_2_gyro;
        expData.IMUData.sensor_2_acc;
        };

    for iPlot = 1:4
        nexttile;
        plot(IMUPlotData{iPlot});
        grid on;
        legend("x", "y", "z");
        axis tight;
    end
end