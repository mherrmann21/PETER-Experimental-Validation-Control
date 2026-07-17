%% Identify system parameters for PETER (tendon-actuated continuum manipulator)
% based on "static" data from static configurations
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Script settings

% Only simulate system with nominal parameters?
SIM_ONLY = 0;

% Experimental data
dataFolder   = fullfile(getRepositoryRootFolder, "data", "experiments", "processed");
dataFileName = "251218_1440_id_data_static_tendon_1_setpoints_combined.mat";

% IMU calibration data
accCalibFile = fullfile(getRepositoryRootFolder, "data", "calibration", "IMUCalib_260717_1307");

% Output folder
saveFolder = fullfile(getRepositoryRootFolder, "data", "identification");

% System & Model settings
usedTendons = [1,2,3];
nSeg = 12;


%% Load experiment data

expData = load(fullfile(dataFolder, dataFileName));

yExp = struct();
yExp.Lc = expData.yLc(usedTendons,:);
yExp.Acc = expData.yAcc;

uSP = expData.u(usedTendons,:);

% Restrict setpoints
uMin = 0;
idxSetpoints = vecnorm(uSP, 2, 1) > uMin;
%idxSetpoints = [1:12, 23:22+12, 45:44+12];

yExp.Lc  = yExp.Lc(:, idxSetpoints);
yExp.Acc = yExp.Acc(:, :, idxSetpoints);
uSP      = uSP(:, idxSetpoints);

nSetpoints = size(uSP,2);
nCables = length(usedTendons);

% Apply accelerometer calibration
yExp.Acc = applyAccelerometerCalibration(yExp.Acc, accCalibFile);

% Plot setpoint data

figure("Name", "Setpoint values");
tiledlayout;
nexttile;
plot(1:nSetpoints, yExp.Lc, "-o");
grid on;
title("Tendon displacement")
xlabel("Setpoint Nr.")

nexttile;
plot(1:nSetpoints, squeeze(yExp.Acc(:,1,:)), "-o");
plot(1:nSetpoints, squeeze(yExp.Acc(:,2,:)), "-o");
grid on;
title("Accelerometer values")
xlabel("Setpoint Nr.")

nexttile;
plot(1:nSetpoints, uSP, "-o");
grid on;
title("Tendon tension")
xlabel("Setpoint Nr.")


%% Get indices of positive and negative tension setpoints

xThreshold = 0;
idx_SP_tdP = find(yExp.Lc > xThreshold);
idx_SP_tdN = find(yExp.Lc < xThreshold);


%% Define Nominal System

links = systemDef_PETER_nominal_reduced("nSeg", nSeg, "usedTendons", usedTendons);
MBSim = MBSimulation(links, "displayInfo", false);

[IMUDef, cableDef, IMUParams] = definePETEROutputs(links);

% Temp tendon friction parameters
%MBSim.MBSys.frameData.d_c_s = ones(1,MBSim.MBSys.nFrames)*1e-3*1e-10;
%MBSim.MBSys.dCDyn = ones(MBSim.MBSys.nInputs,1)*1e-2*0;

MBSysOpt = MBSim.MBSys;
MBSysSym = MBSystemSym(links);
%MBSysSym.frameData.d_c_s = MBSim.MBSys.frameData.d_c_s;
%MBSysSym.dCDyn = ones(MBSim.MBSys.nInputs,1)*1e-2*0;

IDSystemNum = struct;
IDSystemNum.MBSys    = MBSysOpt;
IDSystemNum.IMUDef   = IMUDef;
IDSystemNum.cableDef = cableDef;

IDSystemSym = IDSystemNum;
IDSystemSym.MBSys = MBSysSym;


%% Initial guess: Simulate with measured system inputs

[qSim, ySim, LcOffsetSim] = computeSetPointEqulibria(MBSim, uSP, IMUDef, cableDef);
ySim.Lc = ySim.Lc - LcOffsetSim;

%ySim.Lc = ySim.Lc*1.1;

% Plot nominal outputs
fh = plotStaticSystemOutputComparison(yExp, ySim, "Exp", "Sim Nominal");
fh.Name = "Sim Outputs / Setpoints";

for iT = 1:3
    fh = plotStaticSystemOutputComparison( ...
        yExp, ySim, "Exp", "Sim Nominal", ...
        "plotOverTension", true, "setPointTensions", uSP(iT,:));
    fh.Name = sprintf("Sim Outputs / Tensions T%d", iT);
end


%% Some helper plots

% Plot strain distribution over s
if 0
    iSP = 12;
    xi = MBSysOpt.getLinkDeformations(qSim(:,iSP), 1);
    sNodes = [0;cumsum(MBSysOpt.frameData.l)];

    figure("Name", "Strain Distribution", "NumberTitle", "off");
    stairs(sNodes, [xi(1:3,:), xi(1:3, end)].', '-o');
    grid on;
    xlabel("beam length in m", "Interpreter", "latex");
    ylabel("rotational strain", "Interpreter", "latex");
    legend("$x$", "$y$", "$z$", "interpreter", "latex");
end

% Visualization
if 0
    [fh, vis] = MBSim.visualizeSystemConfig(qSim(:,1));
    vis.linkVis{1}.beamVis.ShowFrames = true;
    vis.linkVis{1}.beamVis.ShowLabels = true;
    axis tight;
    zlim([0,0.7]);
end

if SIM_ONLY
    return;
end


%% Prepare Identification
%% Define Model parameters
 
% Define NLP
opti = casadi.Opti;

calibrationOnly = true;

[IDSystemNLP, IDVars, pVectors] = getParamIDMBSys( ...
    opti, false, calibrationOnly, IDSystemSym, LcOffsetSim);

paramVecNLP = pVectors.NLPVar;


%% Define Equilibrium constraints and system outputs

q_NLP = opti.variable(MBSysOpt.nDoF, nSetpoints);
u_NLP = IDVars.uScale.NLPVar * uSP;

disp("Assembling equilibrium constraints and system outputs...")

[c_Equ, yNLP] = prepareStaticEquVarsParamID(IDSystemNLP, MBSim.simPars, ...
    q_NLP, u_NLP, paramVecNLP, ...
    "useFunctionMap", true);

% Add Equil. equations as constraint
opti.subject_to(c_Equ == 0);


%% Cost function

disp("Assembling cost function...")

% Output weights
wY = opti.parameter(3);

% Regulation weights
wR = opti.parameter(1);


%yLcOffset_NLP = IDVars.LcScale.NLPVar*(yNLP.Lc - IDVars.LcOffset.NLPVar);
yLcOffset_NLP_N = IDVars.LcScaleN.NLPVar .* (yNLP.Lc - IDVars.LcOffset.NLPVar);
yLcOffset_NLP_P = IDVars.LcScaleP.NLPVar .* (yNLP.Lc - IDVars.LcOffset.NLPVar);

fStat = ...
    wY(1) * sumsqr(yNLP.IMUAcc1-squeeze(yExp.Acc(:,1,:))) ...
    + wY(2) * sumsqr(yNLP.IMUAcc2-squeeze(yExp.Acc(:,2,:))) ...
    + wY(3) * sumsqr(yLcOffset_NLP_N(idx_SP_tdN)-yExp.Lc(idx_SP_tdN)) ...
    + wY(3) * sumsqr(yLcOffset_NLP_P(idx_SP_tdP)-yExp.Lc(idx_SP_tdP));

% Regulation term
fR = wR * sumsqr( pVectors.fr .* (paramVecNLP - pVectors.iv));

opti.minimize(fStat + fR);


%% Set initial values and bounds

% Model parameters
IDVarFields = fieldnames(IDVars);
for iVar = 1:length(IDVarFields)
    varStruct = IDVars.(IDVarFields{iVar});

    opti.set_initial(varStruct.NLPVar, varStruct.iv);
    opti.subject_to(varStruct.NLPVar(:) >= varStruct.lb(:));
    opti.subject_to(varStruct.NLPVar(:) <= varStruct.ub(:));
end

opti.set_initial(q_NLP, qSim);

% Weights
% Roughly normalize scales of all outputs
sYAcc = 1/max(abs(yExp.Acc(:)));
sYLc  = 1/max(abs(yExp.Lc(:)));
sVec = [sYAcc,sYAcc,sYLc];

opti.set_value(wY, [1,1,1].*sVec*1e0);
opti.set_value(wR, 0.001);


% Solver settings
p_opts = struct('expand', false);
s_opts = struct('max_iter', 125);
s_opts.linear_solver = 'ma97'; % best for stiff systems

opti.solver('ipopt', p_opts, s_opts);


%% Solve NLP

drawnow;

disp("Solving NLP...")
sol = opti.solve();


%% Get outputs

GET_DEBUG_VALUE = false; % Set to true if solution failed

if GET_DEBUG_VALUE
    solLcOffset = opti.debug.value(IDVars.LcOffset.NLPVar);
    ySolIMUAcc1 = opti.debug.value(yNLP.IMUAcc1);
    ySolIMUAcc2 = opti.debug.value(yNLP.IMUAcc2);
    ySolLcP      = opti.debug.value(yLcOffset_NLP_P);
    ySolLcN      = opti.debug.value(yLcOffset_NLP_N);
else
    solLcOffset = sol.value(IDVars.LcOffset.NLPVar); % ,opti.initial());
    ySolIMUAcc1 = sol.value(yNLP.IMUAcc1);
    ySolIMUAcc2 = sol.value(yNLP.IMUAcc2);
    ySolLcP      = sol.value(yLcOffset_NLP_P);
    ySolLcN      = sol.value(yLcOffset_NLP_N);
end

ySolLc = ySolLcP;
ySolLc(idx_SP_tdN) = ySolLcN(idx_SP_tdN);

% Check outputs of the initial guess
%sol.value(opti.initial())

% Assign to output struct
yOpt = struct();
yOpt.Acc = cat(2, reshape(ySolIMUAcc1, 3, 1, []), reshape(ySolIMUAcc2, 3, 1, []));
yOpt.Lc  = ySolLc;


fh = plotStaticSystemOutputComparison(yExp, yOpt, "Exp", "Opt");
fh.Name = "Opt Outputs / Setpoints";


for iT = 1:3
    fh = plotStaticSystemOutputComparison( ...
        yExp, yOpt, "Exp", "Opt", ...
        "plotOverTension", true, "setPointTensions", uSP(iT,:));
    fh.Name = sprintf("Opt Outputs / Tensions T%d", iT);
end

%% Get solution trajectory

if GET_DEBUG_VALUE
    qSol = opti.debug.value(q_NLP);
else
    qSol = sol.value(q_NLP);
end

%% Get model parameters

disp("Identified parameter variables:")

paramsRel = struct();
for iVar = 1:length(IDVarFields)
    pName = IDVarFields{iVar};
    if GET_DEBUG_VALUE
        paramsRel.(pName) = opti.debug.value(IDVars.(pName).NLPVar);
    else
        paramsRel.(pName) = sol.value(IDVars.(pName).NLPVar);
    end

    fprintf("%s:\n", pName);
    disp(paramsRel.(pName).');
end

paramsRel.LcOffset = solLcOffset;

%% Get numeric MBSystem with identified parameters

IDSystemOpt = getMBSysParamsFromIDVars(IDSystemNum, IDSystemNLP, sol);

MBSysOpt    = IDSystemOpt.MBSys;
cableDefOpt = IDSystemOpt.cableDef;
IMUDefOpt   = IDSystemOpt.IMUDef;

%% Validation simulation with identified parameters

MBSimVal = MBSim;
MBSimVal.MBSys = IDSystemOpt.MBSys;

[qVal, yVal, LcOffsetVal] = computeSetPointEqulibria(MBSimVal, uSP, IMUDefOpt, cableDefOpt);
yVal.Lc = yVal.Lc - solLcOffset;

fh = plotStaticSystemOutputComparison(yExp, yVal, "Exp", "Opt/Val");
fh.Name = "Val Outputs / Setpoints";

fh = plotStaticSystemOutputComparison( ...
    yExp, yVal, "Exp", "Opt/Val", ...
    "plotOverTension", true, "setPointTensions", uSP(1,:));
fh.Name = "Val Outputs / Tensions";


%% Save identified system

saveFileName = sprintf("IDParams_static_%s_nSeg_%d", string(datetime, 'yyMMdd_HHmm'), links(1).nSeg);

disp("Saving results...")
fprintf("Filename: %s\n", saveFileName);

save(fullfile(saveFolder,saveFileName), "MBSysOpt", "IMUDefOpt", "cableDefOpt", "paramsRel", "links");


%% End script
disp("Finished.")
