%% Static Acceleration Calibration
% Estimate static calibration parameters M and b for a calibration model
%       aTrue = M * (aMeas - b)
% from experimental data and save to file.
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich
%
% For background, see:
% https://ieeexplore.ieee.org/document/4655611
% https://www.mathworks.com/matlabcentral/fileexchange/33252-mems-accelerometer-calibration-using-gauss-newton-method
% https://ieeexplore.ieee.org/document/5594974

clear
close all

%% Script settings

dataFolder = fullfile(getRootFolder, "data", "experiments", "processed");
dataFileName = "251218_1440_id_data_static_tendon_1_setpoints_combined.mat";

saveFolder = fullfile(getRootFolder, "data", "calibration");

usedTendons = [1,2,3];


%% Load experiment data

expData = load(fullfile(dataFolder, dataFileName));

yExp = struct();
yExp.Lt = expData.yLt(usedTendons,:);
yExp.Acc = expData.yAcc;

uSP = expData.u(usedTendons,:);

% Restrict setpoints
uMin = 0;
idxSetpoints = vecnorm(uSP, 2, 1) > uMin;
%idxSetpoints = [1:12, 23:22+12, 45:44+12];

yExp.Lt  = yExp.Lt(:, idxSetpoints);
yExp.Acc = yExp.Acc(:, :, idxSetpoints);
uSP      = uSP(:, idxSetpoints);

nIMUs      = size(yExp.Acc,2);
nSetpoints = size(uSP,2);

% Plot setpoint data
figure("Name", "Setpoint values");
tiledlayout;
nexttile;
plot(1:nSetpoints, yExp.Lt, "-o");
grid on;
title("Tendon displacement")
xlabel("Setpoint Nr.")

nexttile;
plot(1:nSetpoints, squeeze(yExp.Acc(:,1,:)), "-o");
hold on;
plot(1:nSetpoints, squeeze(yExp.Acc(:,2,:)), "-o");
grid on;
title("Accelerometer values")
xlabel("Setpoint Nr.")

nexttile;
plot(1:nSetpoints, uSP, "-o");
grid on;
title("Tendon tension")
xlabel("Setpoint Nr.")

% Gravity vector illustrations from raw data
plotGravityVector(yExp.Acc, " (raw)");


%% Optimize in CasADi

Ssym = casadi.SX.sym('SVec', [3,3]);
Osym = casadi.SX.sym('O', [3,1]);

% define error
g = 9.807232; % gravity constant Munich
aSymMeas = casadi.SX.sym('a', [3,1]);
aSymCal  = Ssym * (aSymMeas - Osym);
eSym = sumsqr(aSymCal) - g^2;
eFun = casadi.Function('eFun', {aSymMeas, Ssym, Osym}, {eSym});

accCalib = yExp.Acc;
M = zeros(3,3,2);
b = zeros(3,2);

for iIMU = 1:nIMUs
    opti = casadi.Opti;
    S_NLP = opti.variable(3,3, 'symmetric');
    O_NLP = opti.variable(3,1);

    % Errors for individual setpoints, eq. (4)
    eVecC = cell(nSetpoints,1);
    for iSP = 1:nSetpoints
        eVecC{iSP} = eFun(yExp.Acc(:,iIMU,iSP), S_NLP, O_NLP);
    end
    eVec = horzcat(eVecC{:});

    % Overall error function (5)
    eFunAll = sumsqr(eVec) / nSetpoints;

    opti.minimize(eFunAll);
    opti.set_initial(S_NLP, eye(3));
    opti.set_initial(O_NLP, zeros(3,1));

    opti.solver('ipopt');
    sol = opti.solve;

    % Get solution data
    M(:,:,iIMU) = sol.value(S_NLP);
    b(:,iIMU) = sol.value(O_NLP);
    accCalib(:,iIMU,:) = M(:,:,iIMU)*(squeeze(yExp.Acc(:,iIMU,:)) - b(:,iIMU));
end

%% Plot calibrated values

figure("Name", "IMU Data Comparison");
tiledlayout;
for iIMU = 1:nIMUs
    nexttile;
    plot(1:nSetpoints, squeeze(yExp.Acc(:,iIMU,:)), "-o", "DisplayName", "raw");
    hold on;
    plot(1:nSetpoints, squeeze(accCalib(:,iIMU,:)), "--o", "DisplayName", "calibrated");
    grid on;
    colororder(lines(3));
    title(sprintf("Accelerometer values IMU %d", iIMU));
    xlabel("Setpoint Nr.")
    legend;
end

% Gravity vector illustrations for calibrated data
plotGravityVector(accCalib, " (calibrated)");


%% Save data
saveFileName = sprintf("IMUCalib_%s", string(datetime, 'yyMMdd_HHmm'));

disp("Saving results...")
fprintf("Filename: %s\n", saveFileName);
if ~isfolder(saveFolder)
    mkdir(saveFolder);
end
save(fullfile(saveFolder,saveFileName), "M", "b");


%% End script
disp("Finished.")


%% Local functions
function plotGravityVector(accValues, dataName)
    %% Plot/Illustrate gravity vector from IMU data
    arguments
        % Acceleration values, dimensions (3,nIMUs,nSetpoints)
        accValues   (3,:,:) double

        % Name of the data
        dataName    (1,1) string = ""
    end

    nSetpoints = size(accValues, 3);
    nIMUs      = size(accValues,2);
    g = 9.807232; % gravity constant Munich

    %% Plot gravity-vector magnitude
    figure("Name", "Gravity Vector Magnitude" + dataName)
    plot(vecnorm(squeeze(accValues(:,1,:)))/g)
    hold on;
    plot(vecnorm(squeeze(accValues(:,2,:)))/g)
    grid on;
    xlabel("setpoint nr.");
    ylabel("Measured g");
    legend("IMU 1", "IMU 2");

    %% Gravity vector illustration
    [X,Y,Z] = sphere(50);

    figure("Name", "Gravity Vector Visualization" + dataName);
    tiledlayout;
    for iIMU = 1:nIMUs
        nexttile;
        quiver3( ...
            zeros(nSetpoints,1), zeros(nSetpoints,1), zeros(nSetpoints,1), ...
            squeeze(accValues(1,iIMU,:))/g, squeeze(accValues(2,iIMU,:))/g, ...
            squeeze(accValues(3,iIMU,:))/g, "AutoScale", "off" ...
            );
        hold on;
        surf(X,Y,Z, "FaceAlpha", 0.2, "EdgeAlpha", 0.3)
        axis equal
        title(sprintf("IMU %d", iIMU));
    end
end
