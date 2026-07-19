%% Extract static setpoint data from static setpoint experiments
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all

%% Script settings

% Experiment data folder
dataFolder = fullfile(getRootFolder, "data", "experiments", "raw");

% Experiment data file name
% Run the script for all three tendons!
dataFileName = "251218_1440_id_data_static_tendon_3.mat";

saveFolder = fullfile(getRootFolder, "data", "experiments", "processed");


%% Load and prepare data
expData = load(fullfile(dataFolder, dataFileName));

% Get sample times
toutTens = expData.tendonTensionActual_N.Time - expData.tendonTensionActual_N.Time(1);
toutIMU = expData.IMUData.sensor_1_acc.Time - expData.IMUData.sensor_1_acc.Time(1);
hData = mean(diff(toutTens));


%% Filter data (anti-stiction jitter)
fs = 1/hData;
fc = 12;
[b,a] = butter(4, fc/(fs/2));

expDataFilt = struct();
expDataFilt.tendonTensionTarget = filtfilt(b, a, expData.tendonTensionTarget_N.Data);
expDataFilt.tendonTensionActual = filtfilt(b, a, expData.tendonTensionActual_N.Data);
expDataFilt.acc_1 = filtfilt(b, a, squeeze(expData.IMUData.sensor_1_acc.Data).');
expDataFilt.acc_2 = filtfilt(b, a, squeeze(expData.IMUData.sensor_2_acc.Data).');
expDataFilt.tendonDisplacementActual = filtfilt(b, a, expData.tendonDisplacementActual_m.Data);


%% Find setpoint start times
uSum = sum(expData.tendonTensionTarget_N.Data, 2);

[~, idxSetPointStart] = findpeaks(abs(diff(uSum,2)), ...
    "MinPeakDistance", 10/hData, "MinPeakHeight", 0.05);

% Plot data used to find peaks
figure;
tiledlayout("vertical");
nexttile;
plot(uSum);
title("sum tendon tensions over time")

nexttile;
plot(diff(uSum, 2));
title("Second derivative sum tendon tensions")

% Plot found segment end times
figure;
plot(toutTens, sum(expDataFilt.tendonTensionTarget,2));
hold on;
plot(toutTens(idxSetPointStart), sum(expDataFilt.tendonTensionTarget(idxSetPointStart,:),2), 'o');
title("Identified segment end times");


%% Specify data windows

% Times before the segment end for the data extraction window
tSetpointDataStart = 0.9;
tSetpointDataEnd   = 0.1;

% Exclude setpoint starts that are too close to zero (would result in
% negative setpoint times)
idxSetPointStart = idxSetPointStart(idxSetPointStart > round( tSetpointDataStart / hData ));

% Get indices
idxDataStart = idxSetPointStart - round( tSetpointDataStart / hData );
idxDataEnd   = idxSetPointStart - round( tSetpointDataEnd / hData );

nSetpoints = length(idxDataStart);


%% Plot raw data

% Tendon Data

fhs(1) = figure("Name", "Tendon Data", "NumberTitle", "off");
tiledlayout("vertical");
ax = nexttile;
plot(toutTens, expDataFilt.tendonDisplacementActual);
grid on;
axis tight;
legend(arrayfun(@(x)sprintf("Tendon %d", x), 1:4));
title("Tendon displacement")
hold on;

plot(toutTens(idxDataStart), expDataFilt.tendonDisplacementActual(idxDataStart, :), "o", ...
    "DisplayName", "data window start");
plot(toutTens(idxDataEnd), expDataFilt.tendonDisplacementActual(idxDataEnd, :), "s", ...
    "DisplayName", "data window end");
plot(toutTens(idxSetPointStart), expDataFilt.tendonDisplacementActual(idxSetPointStart, :), "x", ...
        "DisplayName", "setpoint start");

colororder(ax, lines(4));

ax = nexttile;
plot(toutTens, expDataFilt.tendonTensionActual);
hold on;
plot(toutTens, expDataFilt.tendonTensionTarget);
grid on;
axis tight;
legend([ ...
    arrayfun(@(x)sprintf("Tendon %d Actual", x), 1:4),...
    arrayfun(@(x)sprintf("Tendon %d Target", x), 1:4)]);
title("Tendon tensions")

colororder(ax, lines(8));

% Accelerometer Data

fhs(2) = figure("Name", "IMU Data", "NumberTitle", "off");
tiledlayout("vertical");

IMUPlotData = {
    expDataFilt.acc_1;
    expDataFilt.acc_2;
    };

for iPlot = 1:2
    nexttile;
    plot(toutIMU, squeeze(IMUPlotData{iPlot}));
    grid on;
    legend("x", "y", "z");
    axis tight;
end


%% Get mean setpoint values

yLt  = zeros(4,nSetpoints);
yAcc = zeros(3,2,nSetpoints);
u   = zeros(4,nSetpoints);

for iSP = 1:nSetpoints
    idxSetpoint = idxDataStart(iSP):idxDataEnd(iSP);
    yLt(:,iSP) = mean(expDataFilt.tendonDisplacementActual(idxSetpoint,:));
    
    yAcc(:,1,iSP) = mean(expDataFilt.acc_1(idxSetpoint,:));
    yAcc(:,2,iSP) = mean(expDataFilt.acc_2(idxSetpoint,:));
    u(:,iSP)    = mean(expDataFilt.tendonTensionActual(idxSetpoint,:));
end

%% Save data

saveFileName = replace(dataFileName, ".mat", "_setpoints.mat");

save(fullfile(saveFolder, saveFileName), "yLt", "yAcc", "u");


%% Plot setpoint data

figure("Name", "Setpoint values");
tiledlayout;
nexttile;
plot(1:nSetpoints, yLt, "-o");
grid on;
title("Tendon displacement");
xlabel("Setpoint Nr.");

nexttile;
plot(1:nSetpoints, squeeze(yAcc(:,1,:)), "-o");
hold on;
plot(1:nSetpoints, squeeze(yAcc(:,2,:)), "-o");
grid on;
title("Accelerometer values");
xlabel("Setpoint Nr.");

nexttile;
plot(1:nSetpoints, u, "-o");
grid on;
title("Tendon tension");
xlabel("Setpoint Nr.");

%% End script
disp("Finished.")
