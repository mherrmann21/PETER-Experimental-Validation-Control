%% Validate PETER model based on experimental data (static and dynamic)
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Script settings

CREATE_INDIV_PLOTS = 0; % Create individual plots for publication?
SAVE_PLOTS = 1;         % Save the publication plots?

% Compare VI computation time to ODE solvers 
% (and accurately measure computation time?
TIMING_COMPARISON = 0;

% Which tendons to use
usedTendons = [1,2,3];

% Plot save folder
plotSaveDir = fullfile(getRepositoryRootFolder, "results", "validation");

% Accelerometer calibration file
accCalibFile = fullfile(getRepositoryRootFolder, "data", "calibration", "IMUCalib_260717_1307");

% Identified system model
modelParameterFile = fullfile(getRepositoryRootFolder, "data", "identification", "IDParams_static_260717_1309_nSeg_12");

% Dynamic data parameters
dataFolderDyn = fullfile(getRepositoryRootFolder, "data", "experiments", "raw");
dataFileNameDyn = "251218_1440_id_data_dynamic.mat";
tStartOffset = 1.5;
tEnd = 10;
tYShift = 0;
h = 2^-9;

% Static data
dataFolderStat = fullfile(getRepositoryRootFolder, "data", "experiments", "processed");
dataFileNameStat = "251218_1440_id_data_static_tendon_1_setpoints_combined.mat";


%% Load dynamic experiment data

% Simulation time vector (has length nSteps + 1)
nSteps = round( tEnd / h );
tout = (0:h:h*nSteps)';

[tensionsExpDyn, yExpDyn] = getExperimentData(fullfile(dataFolderDyn, dataFileNameDyn), tout, ...
    "tStartOffset", tStartOffset, "tYShift", tYShift,...
    "usedTendons", usedTendons, ...
    "fCTendons", 25, "fCOutput", 27,...
    "useReferenceTensions", false);

[tensionsExpDynRef, ~] = getExperimentData(fullfile(dataFolderDyn, dataFileNameDyn), tout, ...
    "tStartOffset", tStartOffset, "tYShift", tYShift,...
    "usedTendons", usedTendons, ...
    "fCTendons", 20, "fCOutput", 27,...
    "useReferenceTensions", true);

% Apply accelerometer calibration
yExpDyn.IMUAcc = applyAccelerometerCalibration(yExpDyn.IMUAcc , accCalibFile);

% Plot system outputs
fhs_dyn = plotSystemOutputs(yExpDyn, "Exp");


%% Load static experiment data

expDataStat = load(fullfile(dataFolderStat, dataFileNameStat));

yExpStat = struct();
yExpStat.Lc = expDataStat.yLc(usedTendons,:);
yExpStat.Acc = expDataStat.yAcc;

uSP = expDataStat.u(usedTendons,:);

% Restrict setpoints
uMin = 0;
idxSetpoints = vecnorm(uSP, 2, 1) > uMin;
%idxSetpoints = 1:12;
%idxSetpoints = [1:12, 23:22+12, 45:44+12];

yExpStat.Lc  = yExpStat.Lc(:, idxSetpoints);
yExpStat.Acc = yExpStat.Acc(:, :, idxSetpoints);
uSP      = uSP(:, idxSetpoints);

% Apply accelerometer calibration
yExpStat.Acc = applyAccelerometerCalibration(yExpStat.Acc, accCalibFile);


%% Define Nominal System

links = systemDef_PETER_nominal_reduced("nSeg", 12, "usedTendons", usedTendons);
MBSim = MBSimulation(links, "displayInfo", false);
if 0
    % Nominal system
    [IMUDef, cableDef] = definePETEROutputs(links);
    MBSys = MBSim.MBSys;
    MBSysSym = MBSystemSym(links);

    IDstruct.paramsRel.LcScaleP = 1;
    IDstruct.paramsRel.LcScaleN = 1;
    IDstruct.paramsRel.LcOffset = ones(MBSys.nInputs,1)*0.68;
else
    % Load system from identification
    IDstruct = load(modelParameterFile);
    MBSys    = IDstruct.MBSysOpt;
    MBSysSym = MBSystemNum2MBSystemSym(MBSys);
    IMUDef = IDstruct.IMUDefOpt;
    cableDef = IDstruct.cableDefOpt;
    MBSim.MBSys = MBSys;
end

IDSystemNum = struct;
IDSystemNum.MBSys    = MBSys;
IDSystemNum.IMUDef   = IMUDef;
IDSystemNum.cableDef = cableDef;

IDSystemSym = IDSystemNum;
IDSystemSym.MBSys = MBSysSym;

%% Dynamic simulation

% End time
MBSim.simPars.tEnd = tEnd;

% Initial configuration
MBSim.simPars.q0    = zeros(MBSim.MBSys.nDoF,1);
MBSim.simPars.qDot0 = zeros(MBSim.MBSys.nDoF,1);

MBSim.Name = "Dynamic Val.";

MBSim.simPars.uSampleTimes  = tout;
MBSim.simPars.uSampleValues = tensionsExpDyn.';

% Solver settings
MBSim.solver = MBSimIntegratorVarIntBroyden;
MBSim.solver.h = h;
MBSim.solver.JacobianIterationThreshold = 4;
MBSim.solver.errorMargin = 1e-9;
MBSim.solver.aTrapez = 0;
MBSim.solver.accurateTiming = TIMING_COMPARISON;

% Start integration
MBSim = MBSim.simulateSystem;

% Plotting
MBSim.plotAll;

% Animate results
%MBSimVal.animateSimResults("figureName", "AnimVI");

% Compute system outputs
disp("Computing simulation outputs...")
[ySimDyn, LcOffsetSimDyn]  = computeSystemOutputsSim(MBSim, IMUDef, cableDef, "useTendonLengthOffset", false);

% Plot outputs
%fhs = plotSystemOutputs(ySim, "Sim");

% Adjust tendon lengths in simulation data
ySimDyn_Lc_P = IDstruct.paramsRel.LcScaleP .* (ySimDyn.Lc - IDstruct.paramsRel.LcOffset);
ySimDyn_Lc_N = IDstruct.paramsRel.LcScaleN .* (ySimDyn.Lc - IDstruct.paramsRel.LcOffset);
ySimDyn.Lc(ySimDyn.Lc > 0) = ySimDyn_Lc_P(ySimDyn.Lc > 0);
ySimDyn.Lc(ySimDyn.Lc < 0) = ySimDyn_Lc_N(ySimDyn.Lc < 0);

% Comparison
fhs_dyn = plotSystemOutputComparison(yExpDyn, ySimDyn, "Exp", "IG Nominal");


%% Static simulation

[qSimStat, ySimStat, LcOffsetSimStat] = computeSetPointEqulibria(MBSim, uSP, IMUDef, cableDef);
%ySimStat.Lc = ySimStat.Lc - LcOffsetSimStat;

ySimStat_Lc_P = IDstruct.paramsRel.LcScaleP .* (ySimStat.Lc - IDstruct.paramsRel.LcOffset);
ySimStat_Lc_N = IDstruct.paramsRel.LcScaleN .* (ySimStat.Lc - IDstruct.paramsRel.LcOffset);
ySimStat.Lc(ySimStat.Lc > 0) = ySimStat_Lc_P(ySimStat.Lc > 0);
ySimStat.Lc(ySimStat.Lc < 0) = ySimStat_Lc_N(ySimStat.Lc < 0);

% Plot outputs
fh = plotStaticSystemOutputComparison(yExpStat, ySimStat, "Exp", "Sim Nominal");
fh.Name = "Sim Outputs / Setpoints";

for iT = 1:3
    fh = plotStaticSystemOutputComparison( ...
        yExpStat, ySimStat, "Exp", "Sim Nominal", ...
        "plotOverTension", true, "setPointTensions", uSP(iT,:));
    fh.Name = sprintf("Sim Outputs / Tensions T%d", iT);
end

%% Compute RMSE values
disp("RMSE Accelerations:")
disp(rmse(yExpStat.Acc,ySimStat.Acc,3).');

disp("RMSE Tendon Displacements (mm):")
disp(rmse(yExpStat.Lc,ySimStat.Lc,2).'*1e3);



%%
if CREATE_INDIV_PLOTS
    %% Dynamics: Plot inputs / cable tensions

    plotLineWidth = 1.3;

    % Colors for exp/sim comparisons
    colorA = tumColors().TUMBlue4;
    colorB = tumColors().TUMOrange;

    % Cable lengths
    fh_tens = figure( ...
        "Name", "comp_dyn_tensions", ...
        "NumberTitle", "off", ...
        "Theme", "Light");

    tiledlayout("vertical", ...
        "TileSpacing", "tight", "Padding", "tight");
    for iAxis = 1:size(tensionsExpDyn,2)
        ax = nexttile;
        plot(tout, tensionsExpDyn(:,iAxis), "-", "LineWidth", plotLineWidth);
        hold on;
        plot(tout, tensionsExpDynRef(:,iAxis), "-.", "LineWidth", plotLineWidth);
        grid on;
        ylabel(sprintf("$u_{%d}$ in N", iAxis), "Interpreter", "latex");
        ax.TickLabelInterpreter = "latex";
        if iAxis == 1
            legend("reference", "experiment", ...
                "interpreter", "latex", "Location", "northeast", ...
                "IconColumnWidth", 25);
        end
        axis padded;
        xlim(tout([1,end]));
        colororder(ax,[colorA;colorB]);
    end
    xlabel("time $t$ in s", "Interpreter", "latex");


    %% Dynamics: Plot IMU data

    legendNames = ["exp.", "mdl."];

    % IMU Data
    nIMUs = size(yExpDyn.IMUAcc, 2);
    axisStringsAcc = "$a_" + ["x$"; "y$"; "z$"];
    axisStringsGyr = "$\omega_" + ["x$"; "y$"; "z$"];

    fhs_dyn = gobjects(2*nIMUs,1);

    for iIMU = 1:nIMUs
        % Accelerations
        accValuesDynExp = squeeze(yExpDyn.IMUAcc(:,iIMU, :));
        accValuesDynSim = squeeze(ySimDyn.IMUAcc(:,iIMU, :));

        fhs_dyn(iIMU) = figure( ...
            "Name", sprintf("comp_dyn_IMU_%d_acc", iIMU), ...
            "NumberTitle", "off", ...
            "Theme", "Light");
        tiledlayout("vertical", ...
            "TileSpacing", "tight", "Padding", "tight");
        for iAxis = 1:3
            ax = nexttile;

            plot(yExpDyn.tout, accValuesDynExp(iAxis,:), "-", ...
                "LineWidth", plotLineWidth);
            hold on;
            plot(ySimDyn.tout, accValuesDynSim(iAxis,:), "-.", ...
                "LineWidth", plotLineWidth);
            grid on;
            ylabel(axisStringsAcc(iAxis) + " in m/s$^2$", "Interpreter", "latex");
            ax.TickLabelInterpreter = "latex";
            colororder(ax,[colorA;colorB]);

            if iAxis == 1
                legend(legendNames, ...
                    "interpreter", "latex", "Location", "northwest", ...
                    "IconColumnWidth", 25, "BackgroundAlpha", 0.85, ...
                    "Orientation", "horizontal");
            end
            axis padded;
            xlim(tout([1,end]));
        end
        xlabel("time $t$ in s", "Interpreter", "latex");

        % Angular velocity
        gyrValuesDynExp = squeeze(yExpDyn.IMUGyr(:,iIMU, :));
        gyrValuesDynSim = squeeze(ySimDyn.IMUGyr(:,iIMU, :));

        fhs_dyn(iIMU + nIMUs) = figure( ...
            "Name", sprintf("comp_dyn_IMU_%d_gyr", iIMU), ...
            "NumberTitle", "off", ...
            "Theme", "Light");

        tiledlayout("vertical", ...
            "TileSpacing", "tight", "Padding", "tight");
        for iAxis = 1:3
            ax = nexttile;

            plot(yExpDyn.tout, gyrValuesDynExp(iAxis,:), "-", ...
                "LineWidth", plotLineWidth);
            hold on;
            plot(ySimDyn.tout, gyrValuesDynSim(iAxis,:), "-.", ...
                "LineWidth", plotLineWidth);
            grid on;
            ylabel(axisStringsGyr(iAxis) + " in rad/s", "Interpreter", "latex");
            ax.TickLabelInterpreter = "latex";
            colororder(ax,[colorA;colorB]);

            if iAxis == 1
                legend(legendNames, ...
                    "interpreter", "latex", "Location", "best", ...
                    "IconColumnWidth", 25, "BackgroundAlpha", 0.85, ...
                    "Orientation", "horizontal");
            end
            axis padded;
            xlim(tout([1,end]));
        end
        xlabel("time $t$ in s", "Interpreter", "latex");
    end


    %%  Dynamics: Tendon lengths

    fhs_dyn(end+1) = figure( ...
        "Name", "comp_dyn_tendon_lengths", ...
        "NumberTitle", "off", ...
        "Theme", "Light");
    tiledlayout("vertical", ...
        "TileSpacing", "tight", "Padding", "tight");
    for iAxis = 1:size(tensionsExpDyn,2)
        ax = nexttile;

        plot(yExpDyn.tout, yExpDyn.Lc(iAxis,:)*1e3, "-", "LineWidth", plotLineWidth);
        hold on;
        plot(ySimDyn.tout, ySimDyn.Lc(iAxis,:)*1e3, "-.", "LineWidth", plotLineWidth);

        grid on;
        ylabel(sprintf("$\\Delta L_%d$ in mm",iAxis), "Interpreter", "latex");
        ax.TickLabelInterpreter = "latex";
        axis padded;
        xlim(tout([1,end]));
        colororder(ax,[colorA;colorB]);
        if iAxis == 1
            legend(legendNames, ...
                "interpreter", "latex", "Location", "southeast", ...
                "IconColumnWidth", 25, "BackgroundAlpha", 0.85);
        end
    end
    xlabel("time $t$ in s", "Interpreter", "latex");


    %% Static plots

    lSt1 = "-";
    lSt2 = "-..";

    nSetpoints = size(ySimStat.Lc, 2);
    nCables = size(yExpDyn.Lc, 1);

    nTendons = size(ySimStat.Lc,1);
    fhs_stat_imu = gobjects(nTendons, nIMUs);
    fhs_stat_L   = gobjects(nTendons,1);

    for iT = 1:nTendons

        idxSetPoints_iT = (iT-1)*(nSetpoints/nTendons)+1:iT*nSetpoints/nTendons;

        uSP_iT = uSP(iT, idxSetPoints_iT);
        % Accelerations
        drawLegends = [1, 0];
        for iIMU = 1:nIMUs

            accValuesExp = squeeze(yExpStat.Acc(:,iIMU, idxSetPoints_iT));
            accValuesSim = squeeze(ySimStat.Acc(:,iIMU, idxSetPoints_iT));

            fhs_stat_imu(iT,iIMU) = figure( ...
                "Name", sprintf("comp_stat_t%d_IMU_%d_acc", iT, iIMU), ...
                "NumberTitle", "off", ...
                "Theme", "Light");

           tiledlayout("vertical", ...
                "TileSpacing", "tight", "Padding", "tight");
            for iAxis = 1:size(tensionsExpDyn,2)
                ax = nexttile;

                plot(uSP_iT, accValuesExp(iAxis,:), ...
                    ".-", "LineWidth", plotLineWidth);
                hold on;
                plot(uSP_iT, accValuesSim(iAxis,:), ...
                    ".-.", "LineWidth", plotLineWidth);

                grid on;
                ylabel(axisStringsAcc(iAxis) + " in m/s$^2$", "Interpreter", "latex");
                colororder(ax,[colorA;colorB]);

                axis padded
                ax.TickLabelInterpreter = "latex";
                if iAxis == 1
                    legend(legendNames, ...
                        "interpreter", "latex", "Location", "best", ...
                        "IconColumnWidth", 25, "BackgroundAlpha", 0.85);
                end
            end
            xlabel(sprintf("tension $u_%d$ in N", iT), "Interpreter", "latex");
        end

        % Tendon displacement
        fhs_stat_L(iT) = figure( ...
            "Name", sprintf("comp_stat_t%d_tendon_lengths", iT), ...
            "NumberTitle", "off", ...
            "Theme", "Light");
        tiledlayout("vertical", ...
            "TileSpacing", "tight", "Padding", "tight");
        for iAxis = 1:size(tensionsExpDyn,2)
            ax = nexttile;

            plot(uSP_iT, yExpStat.Lc(iAxis,idxSetPoints_iT)*1e3, ".-", ...
                "LineWidth", plotLineWidth);
            hold on;
            plot(uSP_iT, ySimStat.Lc(iAxis,idxSetPoints_iT)*1e3, ".-.", ...
                "LineWidth", plotLineWidth);

            grid on;
            ylabel(sprintf("$\\Delta L_%d$ in mm",iAxis), "Interpreter", "latex");
            colororder(ax,[colorA;colorB]);
            axis padded
            ax.TickLabelInterpreter = "latex";

            if iAxis == 1
                legend(legendNames, ...
                    "interpreter", "latex", "Location", "best", ...
                    "IconColumnWidth", 25, "BackgroundAlpha", 0.85);
            end
        end
        xlabel(sprintf("tension $u_%d$ in N", iT), "Interpreter", "latex");
    end

    %% Save plots

    if SAVE_PLOTS
        fhsAll = [
            fhs_stat_L
            fhs_stat_imu(:)
            ];

        if ~isfolder( plotSaveDir )
            mkdir( plotSaveDir );
        end

        drawnow;

        pdfWidth = 7.6*28.346; % width in pt
        pdfAspectRatio = 1.2;

        % Add. margin: [left, right, bottom, top]
        saveFigureArray(fh_tens, plotSaveDir, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatio);
        saveFigureArray(fhs_dyn, plotSaveDir, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatio, ...
            "additionalPDFMargin", [0,0,0,0.3]);
        saveFigureArray(fhsAll, plotSaveDir, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatio, ...
            "additionalPDFMargin", [0,0,0,0.3]);
    end
end

%% Integration with ODE solver

if TIMING_COMPARISON
    MBSimODE = MBSim;
    %MBSimODE.simPars.tEnd = 10;

    % Solver settings
    MBSimODE.solver = MBSimIntegratorODEDirect;
    MBSimODE.solver.odeObject.AbsoluteTolerance = 1e-3;
    MBSimODE.solver.odeObject.RelativeTolerance = 1e-2;
    MBSimODE.solver.accurateTiming = TIMING_COMPARISON;

    % Integration with two different solvers
    disp("Integration with ode15s:")
    MBSimODE.solver.odeObject.Solver = "ode15s";
    MBSimODE = MBSimODE.simulateSystem;

    disp("Integration with ode23t:")
    MBSimODE.solver.odeObject.Solver = "ode23t";
    MBSimODE = MBSimODE.simulateSystem;

    % Plotting
    %MBSimODE.plotAll;
    %MBSimODE = MBSimODE.computeEnergies;
    %plotEnergies(MBSimODE.simRes);

    % Animate results
    %MBSimODE.animateSimResults("figureName", "AnimODE");
end


%% End script
disp("Finished.")