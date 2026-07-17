%% Validate PETER model / trajectory tracking (dynamic)
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Script settings

SAVE_PLOTS = 1;
CREATE_INDIV_PLOTS = true;

SIMULATE_ODE_SOLVER = 0;

% Plot save folder
plotSaveDir = fullfile(getRepositoryRootFolder, "results", "tracking");

% Accelerometer calibration file
accCalibFile = fullfile(getRepositoryRootFolder, "data", "calibration", "IMUCalib_260717_1307");

% Identified system model
modelParameterFile = fullfile(getRepositoryRootFolder, "data", "identification", "IDParams_static_260717_1309_nSeg_12");

% Trajectory test
dataFolderDyn = fullfile(getRepositoryRootFolder, "data", "experiments", "raw");
dataFileNameDyn = "251218_1639_tracking_exp_data.mat";
tStartOffset = 5;
tEnd = 6;
tYShift = 0;
usedTendons = [1,2,3];
h = 2^-9;

% File with the generated trajectory data
trajDataFile = fullfile(getRepositoryRootFolder, "data", "trajectories", "trajectory_251218_1610_nSeg_8");


%% Load dynamic experiment data

% Simulation time vector (has length nSteps + 1)
nStepsExp = round( tEnd / h );
tout = (0:h:h*nStepsExp)';

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
plotSystemOutputs(yExpDyn, "Exp");


%% Load OCP trajectory data for comparison

OCPData = load(trajDataFile);
OCP = OCPData.OCP;
tOffsetOCP = 1.057;

% Extend OCP trajectory
toutOCPExt = 0:OCPData.h:tout(end);
uOCPInput = [OCPData.u_sol(:,1), OCPData.u_sol, OCPData.u_sol(:,end)];
toutOCPInput = [OCPData.tout(1); OCPData.tout+tOffsetOCP; tout(end)];

uOCPExt = interp1(toutOCPInput, uOCPInput.', toutOCPExt);

figure;
plot(tout, tensionsExpDynRef, "DisplayName", "Reference Values Experiment");
hold on;
plot(OCPData.tout+tOffsetOCP, OCPData.u_sol, "-o", "DisplayName", "OCP Values Original");
plot(toutOCPExt, uOCPExt, '-x', "DisplayName", "OCP Values Extended");
grid on;
title("Comparison OCP input trajectory / Exp reference trajectory");
legend;

%% Define Nominal System

links = systemDef_PETER_nominal_reduced("nSeg", 8, ...
    "usedTendons", usedTendons, "d", ones(6,1)*25.0e-3);
MBSim = MBSimulation(links, "displayInfo", false);
if 1
    % Nominal system
    %[IMUDef, cableDef] = definePETEROutputs(links);
    MBSys = MBSim.MBSys;
    MBSysSym = MBSystemSym(links);

    IDstruct.paramsRel.LcScaleP = 1;
    IDstruct.paramsRel.LcScaleN = 1;
    IDstruct.paramsRel.LcOffset = ones(MBSys.nInputs,1)*0.68;
else
    % Load full system from identification
    IDstruct = load(modelParameterFile);
    MBSys    = IDstruct.MBSysOpt;
    MBSysSym = MBSystemNum2MBSystemSym(MBSys);
    IMUDef = IDstruct.IMUDefOpt;
    cableDef = IDstruct.cableDefOpt;
    MBSim.MBSys = MBSys;
end

% Load output definition from identification
IDstructExp = load(modelParameterFile);
IMUDef = IDstructExp.IMUDefOpt;
cableDef = IDstructExp.cableDefOpt;

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
MBSim.simPars.uSampleValues = tensionsExpDynRef.';

% Solver settings
MBSim.solver = MBSimIntegratorVarIntBroyden;
MBSim.solver.h = h;
MBSim.solver.JacobianIterationThreshold = 5;
MBSim.solver.errorMargin = 1e-9;
MBSim.solver.aTrapez = 0;

% Start integration
MBSim = MBSim.simulateSystem;

% Plotting
%MBSimVal.plotAll;

% Animate results
%MBSimVal.animateSimResults("figureName", "AnimVI");

% Compute system outputs
disp("Computing simulation outputs...")
[ySimDyn, LcOffsetSimDyn]  = computeSystemOutputsSim(MBSim, IMUDef, cableDef, "useTendonLengthOffset", false);

% Plot outputs
%fhs = plotSystemOutputs(ySim, "Sim");

% Adjust tendon lengths in simulation data
IDstruct.paramsRel.LcOffset = ySimDyn.Lc(:,350) - yExpDyn.Lc(:,350);


ySimDyn_Lc_P = IDstructExp.paramsRel.LcScaleP .* (ySimDyn.Lc - IDstruct.paramsRel.LcOffset);
ySimDyn_Lc_N = IDstructExp.paramsRel.LcScaleN .* (ySimDyn.Lc - IDstruct.paramsRel.LcOffset);
ySimDyn.Lc(ySimDyn.Lc > 0) = ySimDyn_Lc_P(ySimDyn.Lc > 0);
ySimDyn.Lc(ySimDyn.Lc < 0) = ySimDyn_Lc_N(ySimDyn.Lc < 0);


% Compute outputs from OCP
[yOCP, LcOffsetOCP]  = computeSystemOutputsSim(OCPData.MBSimCasadi, IMUDef, cableDef, "useTendonLengthOffset", false);
yOCP_Lc_P = IDstructExp.paramsRel.LcScaleP .* (yOCP.Lc - IDstruct.paramsRel.LcOffset);
yOCP_Lc_N = IDstructExp.paramsRel.LcScaleN .* (yOCP.Lc - IDstruct.paramsRel.LcOffset);
yOCP.Lc(yOCP.Lc > 0) = yOCP_Lc_P(yOCP.Lc > 0);
yOCP.Lc(yOCP.Lc < 0) = yOCP_Lc_N(yOCP.Lc < 0);
yOCP.tout = yOCP.tout + tOffsetOCP;

% Comparisons
fhs_dyn_ExpMdl = plotSystemOutputComparison(yExpDyn, ySimDyn, "Exp", "Model");
fhs_dyn_MdlOCP = plotSystemOutputComparison(ySimDyn, yOCP, "Sim", "OCP");
fhs_dyn_ExpOCP = plotSystemOutputComparison(yExpDyn, yOCP, "Exp", "OCP");



%% Create detailed plots for thesis

if CREATE_INDIV_PLOTS
    plotLineWidth = 1.3;
    pdfWidth = 7.6*28.346; % width in pt
    pdfAspectRatio = 1.2;

    %% Plot and Visualize Reference Trajectory

    % Compute TCP trajectory from OCP
    x_TCP_traj_OCP = zeros(3,size(OCPData.q_sol,2));
    for iStep = 1:size(OCPData.q_sol,2)
        gStep = MBSim.MBSys.computeFwdKin(OCPData.q_sol(:,iStep));
        g_TCP = gStep(:,:,MBSim.MBSys.indexTCPFrame)*MBSim.MBSys.g_B_TCP;
        x_TCP_traj_OCP(:,iStep) = g_TCP(1:3, 4);
    end

    % Plot over time
    x_TCP_traj_ref = OCP.x_TCP_traj;
    [x_TCP_traj_ref_dt, x_TCP_traj_ref_ddt] = diff2ndOrder(x_TCP_traj_ref, OCP.h);
    [x_TCP_traj_OCP_dt, x_TCP_traj_OCP_ddt] = diff2ndOrder(x_TCP_traj_OCP, OCP.h);

    colors3 = [
        tumColors().TUMBlue4;
        tumColors().TUMBlue2;
        tumColors().TUMOrange;
        ];

    fh = figure("Name", "TCP Trajectory Desired", ...
        "NumberTitle", "off", "Theme", "Light");

    tiledlayout("vertical", "TileSpacing", "tight", "Padding", "compact");

    ax = nexttile;
    plot(OCP.tout, x_TCP_traj_ref, "LineWidth", plotLineWidth);
    %hold on;
    %plot(OCP.tout, x_TCP_traj_OCP, "--", "LineWidth", plotLineWidth);
    grid on;
    ylabel("$x_{\mathrm{TCP}}$ in m", "Interpreter", "latex");
    %xlabel("time $t$ in s", "Interpreter", "latex");
    legend("$x$", "$y$", "$z$", ...
        "interpreter", "latex", "Location", "southeast", ...
        "IconColumnWidth", 10, "Orientation", "horizontal");
    axis padded;
    xlim([OCP.tout(1), OCP.tout(end)]);
    ax.TickLabelInterpreter = "latex";
    ax.ColorOrder = colors3;


    ax = nexttile;
    plot(OCP.tout, x_TCP_traj_ref_dt, "LineWidth", plotLineWidth);
    %hold on;
    %plot(OCP.tout, x_TCP_traj_OCP_dt, "LineWidth", plotLineWidth);
    grid on;
    ylabel("$\dot{x}_{\mathrm{TCP}}$ in m/s", "Interpreter", "latex");
    %xlabel("time $t$ in s", "Interpreter", "latex");
    xlim([OCP.tout(1), OCP.tout(end)]);
    ax.TickLabelInterpreter = "latex";
    ax.ColorOrder = colors3;

    ax = nexttile;
    plot(OCP.tout, x_TCP_traj_ref_ddt, "LineWidth", plotLineWidth);
    %hold on;
    %plot(OCP.tout, x_TCP_traj_OCP_ddt, "LineWidth", plotLineWidth);
    grid on;
    ylabel("$\ddot{x}_{\mathrm{TCP}}$ in m/$\mathrm{s}^2$", "Interpreter", "latex");
    xlabel("time $t$ in s", "Interpreter", "latex");
    xlim([OCP.tout(1), OCP.tout(end)]);
    ax.TickLabelInterpreter = "latex";
    ax.ColorOrder = colors3;

    if SAVE_PLOTS
        pdfAspectRatioRefTraj = 0.9;1.33;
        saveFigureArray(fh, plotSaveDir, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatioRefTraj);
    end


    %% Reference trajectory 3D visualization

    fh3D = figure("Name", "TCP Trajectory Vis", ...
        "NumberTitle", "off", "Theme", "Light");
    init3Dplot("createFigure", false);

    % Visualize manipulator
    [~, vis] = MBSim.visualizeSystemConfig(OCP.q0, "createFigure", false);
    vis.cSysI.h_nameLabel.Visible = "off";
    vis.linkVis(1).cSysJ.Visible = "off";
    vis.linkVis(1).cSysTCP.h_nameLabel.Visible = "off";

    % 3D Trajectory
    plot3(x_TCP_traj_ref(1,:), x_TCP_traj_ref(2,:), x_TCP_traj_ref(3,:), ...
        "LineWidth", 2, "Color", colors3(3,:));

    xlim([ ...
        min([0, min(x_TCP_traj_ref(1,:))])-0.05, ...
        max([0, max(x_TCP_traj_ref(1,:))])+0.05, ...
        ]);
    ylim([ ...
        min([0, min(x_TCP_traj_ref(2,:))])-0.05, ...
        max([0, max(x_TCP_traj_ref(2,:))])+0.05, ...
        ]);
    zlim([ ...
        min([0, min(x_TCP_traj_ref(3,:))]), ...
        max([0.8, max(x_TCP_traj_ref(3,:))])+0.05, ...
        ]);

    % Projections on the coordinate axes
    projLineWidth = 1.5;
    projColor = ones(3,1)*0.8;

    plot3(x_TCP_traj_ref(1,:)*0+fh3D.CurrentAxes.XLim(1), ...
        x_TCP_traj_ref(2,:), ...
        x_TCP_traj_ref(3,:), ...
        "LineWidth", projLineWidth, "Color", projColor);
    plot3(x_TCP_traj_ref(1,:), ...
        x_TCP_traj_ref(2,:)*0+fh3D.CurrentAxes.YLim(1), ...
        x_TCP_traj_ref(3,:), ...
        "LineWidth", projLineWidth, "Color", projColor);
    plot3(x_TCP_traj_ref(1,:), ...
        x_TCP_traj_ref(2,:), ...
        x_TCP_traj_ref(3,:)*0+fh3D.CurrentAxes.ZLim(1), ...
        "LineWidth", projLineWidth, "Color", projColor);

    view(140,35);
    xlabel("$x$ in m", "Interpreter", "latex");
    ylabel("$y$ in m", "Interpreter", "latex");
    zlabel("$z$ in m", "Interpreter", "latex");
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    box on;

    if SAVE_PLOTS
        drawnow;

        pdfAspectRatioVis = 0.9;

        saveFigureArray(fh3D, plotSaveDir, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatioVis);
    end

    %% Inputs and Coordinates: Initial guess and solution

    B = OCP.getInputSplineBasisMatrix;
    u_init = (B*OCPData.u_init_z.').';

    uData = {u_init, OCPData.u_sol};
    qData = {OCPData.q_init, OCPData.q_sol};
    plotNames = ["IG", "sol"];

    % qColors = nebula(size(OCPData.q_init,1));
    % qColors = crameri('romaO', size(OCPData.q_init,1)+1);
    qColors = tumBlueMap(size(OCPData.q_init,1));


    fhs_u = gobjects(2,1);
    fhs_q = gobjects(2,1);
    for iPlot = 1:2
        % Inputs
        fhs_u(iPlot) = figure("Name", "OCP inputs " + plotNames(iPlot), ...
            "NumberTitle", "off", "Theme", "Light");

        ph1 = plot(OCP.tout([1,end]), [1,1]*OCP.uMin(1), "k--", "LineWidth", 0.7);
        hold on;
        ph2 = plot(OCP.tout, uData{iPlot}, "LineWidth", plotLineWidth);
        grid on;
        ylabel("tension $u$ in N", "Interpreter", "latex");
        xlabel("time $t$ in s", "Interpreter", "latex");
        if iPlot == 1
            legend([ph2; ph1], ...
                [arrayfun(@(x) sprintf("$u_%d$", x), 1:size(u_init,1)), ...
                "$u_{\mathrm{min}}$"], ...
                "interpreter", "latex", "Location", "northwest", ...
                "IconColumnWidth", 15);
        end
        ylim([0, 20]);
        colororder([zeros(1,3); colors3]);

        ax = gca;
        ax.TickLabelInterpreter = "latex";
        xlim([OCP.tout(1), OCP.tout(end)]);

        % Coordinates
        [q_dt, q_ddt] = diff2ndOrder(qData{iPlot}, OCP.h);

        fhs_q(iPlot) = figure("Name", "OCP coordinates " + plotNames(iPlot), ...
            "NumberTitle", "off", "Theme", "Light");

        tl = tiledlayout("vertical", "TileSpacing", "tight", "Padding", "tight");
        ax = nexttile;

        plot(OCP.tout, qData{iPlot}, "LineWidth", plotLineWidth);
        grid on;
        ylabel("$q$", "Interpreter", "latex");
        %xlabel("time $t$ in s", "Interpreter", "latex");
        % legend("$x$", "$y$", "$z$", ...
        %     "interpreter", "latex", "Location", "southeast", ...
        %     "IconColumnWidth", 10, "Orientation", "horizontal");
        ax.TickLabelInterpreter = "latex";
        xlim([OCP.tout(1), OCP.tout(end)]);
        colororder(ax, qColors);
        ylim([-4, 4]);

        ax = nexttile;
        plot(OCP.tout, q_dt, "LineWidth", plotLineWidth);
        grid on;
        ylabel("$\dot{q}$", "Interpreter", "latex");
        %xlabel("time $t$ in s", "Interpreter", "latex");
        ax.TickLabelInterpreter = "latex";
        xlim([OCP.tout(1), OCP.tout(end)]);
        colororder(ax, qColors);
        ylim([-5, 5]);

        ax = nexttile;
        plot(OCP.tout, q_ddt, "LineWidth", plotLineWidth);
        grid on;
        ylabel("$\ddot{q}$", "Interpreter", "latex");
        xlabel("time $t$ in s", "Interpreter", "latex");
        ax.TickLabelInterpreter = "latex";
        xlim([OCP.tout(1), OCP.tout(end)]);
        colororder(ax, qColors);
        ylim([-10, 10]);
    end


    if SAVE_PLOTS
        pdfAspectRatioQ = 0.8;
        pdfAspectRatioU = 3/2;

        saveFigureArray(fhs_q, plotSaveDir, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatioQ);
        saveFigureArray(fhs_u, plotSaveDir, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatioU);
    end



    %% Plot IMU data

    % Colors for exp/sim comparisons
    colorA = tumColors().TUMBlue4;
    colorB = tumColors().TUMOrange;

    legendNames = ["exp.", "mdl."];
    nameExp = "Exp.";
    nameSim = "OCP";

    % IMU Data
    nIMUs = size(yExpDyn.IMUAcc, 2);
    axisStrings = ["$x$"; "$y$"; "$z$"];
    axisStringsAcc = "$a_" + ["x$"; "y$"; "z$"];
    axisStringsGyr = "$\omega_" + ["x$"; "y$"; "z$"];

    fhs_dyn = gobjects(2*nIMUs,1);

    for iIMU = 1:nIMUs
        % Accelerations
        accValuesDynExp = squeeze(yExpDyn.IMUAcc(:,iIMU, :));
        accValuesDynSim = squeeze(yOCP.IMUAcc(:,iIMU, :));

        fhs_dyn(iIMU) = figure( ...
            "Name", sprintf("comp_dyn_IMU_%d_acc", iIMU), ...
            "NumberTitle", "off", ...
            "Theme", "Light");
        tiledlayout("vertical", ...
            "TileSpacing", "tight", "Padding", "tight");
        for iAxis = 1:size(tensionsExpDyn,2)
            ax = nexttile;

            plot(yExpDyn.tout-tOffsetOCP, accValuesDynExp(iAxis,:), "-", ...
                "LineWidth", plotLineWidth);
            hold on;
            plot(yOCP.tout-tOffsetOCP, accValuesDynSim(iAxis,:), "-", ...
                "LineWidth", plotLineWidth*1.2);
            grid on;

            ylabel(axisStringsAcc(iAxis) + " in m/s$^2$", ...
                "Interpreter", "latex");
            colororder(ax,[colorA;colorB]);
            ax.TickLabelInterpreter = "latex";
            axis padded;
            xlim(yExpDyn.tout([1,end])-tOffsetOCP);

            if iAxis == 1
                legend(legendNames, ...
                    "Interpreter", "latex", ...
                    "Location", "southwest", ...
                    "Orientation", "horizontal", ...
                    "IconColumnWidth", 18, ...
                    "BackgroundAlpha", 0.85);
            end
        end
        xlabel("time $t$ in s", "Interpreter", "latex");


        % Angular velocity
        gyrValuesDynExp = squeeze(yExpDyn.IMUGyr(:,iIMU, :));
        gyrValuesDynSim = squeeze(yOCP.IMUGyr(:,iIMU, :));

        fhs_dyn(iIMU + nIMUs) = figure( ...
            "Name", sprintf("comp_dyn_IMU_%d_gyr", iIMU), ...
            "NumberTitle", "off", ...
            "Theme", "Light");

        tiledlayout("vertical", ...
            "TileSpacing", "tight", "Padding", "tight");
        for iAxis = 1:3
            ax = nexttile;
            plot(yExpDyn.tout-tOffsetOCP, gyrValuesDynExp(iAxis,:), "-", ...
                "LineWidth", plotLineWidth);
            hold on;
            plot(yOCP.tout-tOffsetOCP, gyrValuesDynSim(iAxis,:), "-", ...
                "LineWidth", plotLineWidth*1.2);
            grid on;
            ylabel(axisStringsGyr(iAxis) + " in rad/s", "Interpreter", "latex");
            colororder(ax,[colorA;colorB]);
            ax.TickLabelInterpreter = "latex";
            axis padded;
            xlim(yExpDyn.tout([1,end])-tOffsetOCP);

            % if iIMU == 1 && iAxis == 1
            %     legend(legendNames, ...
            %         "Interpreter", "latex", "Location", "northwest", ...
            %         "Orientation", "horizontal", ...
            %         "IconColumnWidth", 18, ...
            %         "BackgroundAlpha", 0.85);
            % end
        end
        xlabel("time $t$ in s", "Interpreter", "latex");
    end


    %% Tendon lengths

    fhs_dyn(end+1) = figure( ...
        "Name", "comp_dyn_tendon_lengths", ...
        "NumberTitle", "off", ...
        "Theme", "Light");
    tiledlayout("vertical", ...
        "TileSpacing", "tight", "Padding", "tight");
    for iAxis = 1:size(tensionsExpDyn,2)
        ax = nexttile;
        plot(yExpDyn.tout-tOffsetOCP, yExpDyn.Lc(iAxis,:)*1e3, "-", ...
            "LineWidth", plotLineWidth);
        hold on;
        plot(yOCP.tout-tOffsetOCP, yOCP.Lc(iAxis,:)*1e3, "-", ...
            "LineWidth", plotLineWidth);
        grid on;
        ylabel(sprintf("$\\Delta L_%d$ in mm", iAxis), "Interpreter", "latex");

        colororder(ax,[colorA;colorB]);
        ax.TickLabelInterpreter = "latex";

        axis padded;
        xlim(yExpDyn.tout([1,end])-tOffsetOCP);
        if iAxis == 1
            legend(ax, legendNames, ...
                "Interpreter", "latex", "Location", "northeast", ...
                "IconColumnWidth", 15);
        end
    end
    xlabel("time $t$ in s", "Interpreter", "latex");


    %% Tendon tensions

    fhs_dyn(end+1) = figure( ...
        "Name", "comp_dyn_tendon_tensions", ...
        "NumberTitle", "off", ...
        "Theme", "Light");
    tiledlayout("vertical", ...
        "TileSpacing", "tight", "Padding", "tight");
    for iAxis = 1:size(tensionsExpDyn,2)
        ax = nexttile;
        plot(yExpDyn.tout-tOffsetOCP, tensionsExpDynRef(:,iAxis).', "-", ...
            "LineWidth", plotLineWidth);
        hold on;
        plot(yExpDyn.tout-tOffsetOCP, tensionsExpDyn(:,iAxis).', "-.", ...
            "LineWidth", plotLineWidth);
        grid on;
        ylabel(sprintf("$u_%d$ in N", iAxis), "Interpreter", "latex");

        colororder(ax,[colorA;colorB]);
        ax.TickLabelInterpreter = "latex";

        axis padded;
        xlim(yExpDyn.tout([1,end])-tOffsetOCP);
        if iAxis == 1
            legend(ax, "reference", "experiment", ...
                "Interpreter", "latex", "Location", "southeast", ...
                "IconColumnWidth", 15);
        end
    end
    xlabel("time $t$ in s", "Interpreter", "latex");

    % Save plots
    if SAVE_PLOTS
        saveFigureArray(fhs_dyn, plotSaveDir, ...
            "saveFig", true, "saveJPEG", true, "savePDF", true, ...
            "pdfWidth", pdfWidth, "pdfAspectRatio", pdfAspectRatio, ...
            "additionalPDFMargin", [0,0,0,0.3]);
    end


end

%% Integration with ODE solver

if SIMULATE_ODE_SOLVER
    MBSimODE = MBSim;
    %MBSimODE.simPars.tEnd = 10;

    % Solver settings
    MBSimODE.solver = MBSimIntegratorODEDirect;
    MBSimODE.solver.odeObject.AbsoluteTolerance = 1e-3;
    MBSimODE.solver.odeObject.RelativeTolerance = 1e-2;

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
