%% Identify system parameters for PETER (tendon-actuated continuum manipulator)
% based on combined data from both static setpoints and dynamic motions
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Script settings

SIM_ONLY = false;

% Experimental data
dataFolderDyn   = fullfile(getRepositoryRootFolder, "data", "experiments", "raw");
dataFileNameDyn = "251112_1607_id_data_dynamic.mat";

dataFolderStat   = fullfile(getRepositoryRootFolder, "data", "experiments", "processed");
dataFileNameStat = "251112_1607_id_data_static_tendon_1_setpoints_combined.mat";

% IMU calibration data
accCalibFile = fullfile(getRepositoryRootFolder, "data", "calibration", "IMUCalib_260717_1307");

% Output folder
saveFolder = fullfile(getRepositoryRootFolder, "data", "identification");

% System and model settings
usedTendons = [1,2,3];
nSeg = 5;

% Dynamic data parameters
tStartOffset = 0;
tStartYError = 0.02;
tEnd = 9;
tYShift = 0;

% Time step
h = 2^-7;

%% Load dynamic experiment data

% Simulation time vector (has length nSteps + 1)
nSteps = round( tEnd / h );
tout = (0:h:h*nSteps)';

[tensionsExpDyn, yExpDyn] = getExperimentData(fullfile(dataFolderDyn, dataFileNameDyn), tout, ...
    "tStartOffset", tStartOffset, "tYShift", tYShift,...
    "usedTendons", usedTendons, ...
    "fCTendons", 25, "fCOutput", 27,...
    "useReferenceTensions", false);

% Apply accelerometer calibration
yExpDyn.IMUAcc = applyAccelerometerCalibration(yExpDyn.IMUAcc, accCalibFile);

% Plot system outputs
plotSystemOutputs(yExpDyn, "Exp");


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

yExpStat.Lc  = yExpStat.Lc(:, idxSetpoints);
yExpStat.Acc = yExpStat.Acc(:, :, idxSetpoints);
uSP          = uSP(:, idxSetpoints);

% Apply accelerometer calibration
yExpStat.Acc = applyAccelerometerCalibration(yExpStat.Acc, accCalibFile);


%% Define Nominal System

links = systemDef_PETER_nominal_reduced("nSeg", nSeg, "usedTendons", usedTendons);
MBSim = MBSimulation(links, "displayInfo", false);

[IMUDef, cableDef] = definePETEROutputs(links);

% Temp tendon friction parameters
%MBSim.MBSys.frameData.d_c_s = ones(1,MBSim.MBSys.nFrames)*5e-1*0;
%MBSim.MBSys.dCDyn = ones(MBSim.MBSys.nInputs,1)*1e-2*0;

MBSys = MBSim.MBSys;
MBSysSym = MBSystemSym(links);

IDSystemNum = struct;
IDSystemNum.MBSys    = MBSys;
IDSystemNum.IMUDef   = IMUDef;
IDSystemNum.cableDef = cableDef;

IDSystemSym = IDSystemNum;
IDSystemSym.MBSys = MBSysSym;


%% Specify Simulation Parameters

% End time
MBSim.simPars.tEnd = tEnd;

% Initial configuration
MBSim.simPars.q0    = zeros(MBSim.MBSys.nDoF,1);
MBSim.simPars.qDot0 = zeros(MBSim.MBSys.nDoF,1);
%MBSim.simPars.q0(1) = 1;

% Visualize initial config
MBSim.visualizeSystemConfig(MBSim.simPars.q0, "figureName", "visInitConf");
title("Initial Configuration")


%% Initial guess: Simulate with measured system inputs

MBSimIG = MBSim;
MBSimIG.Name = "IG Nominal";

MBSimIG.simPars.uSampleTimes  = tout;
MBSimIG.simPars.uSampleValues = tensionsExpDyn.';

% Solver settings
MBSimIG.solver = MBSimIntegratorVarIntBroyden;
MBSimIG.solver.h = h;
MBSimIG.solver.JacobianIterationThreshold = 5;
MBSimIG.solver.errorMargin = 1e-11;
MBSimIG.solver.aTrapez = 0; % 1st-order dissipation for tests

% Start integration
MBSimIG = MBSimIG.simulateSystem;

% Plotting
%MBSimVal.plotAll;

% Animate results
%MBSimVal.animateSimResults("figureName", "AnimVI");


% Compute system outputs
disp("Computing simulation outputs...")
[ySimDyn, LcOffsetSimDyn]  = computeSystemOutputsSim(MBSimIG, IMUDef, cableDef);

% Plot outputs
% plotSystemOutputs(ySim, "Sim");

% Comparison
plotSystemOutputComparison(yExpDyn, ySimDyn, "Exp", "IG Nominal");


%% Initial guess statics: Simulate with measured system inputs

[qSimStat, ySimStat, LcOffsetSimStat] = computeSetPointEqulibria(MBSim, uSP, IMUDef, cableDef);
ySimStat.Lc = ySimStat.Lc - LcOffsetSimStat;

% Plot nominal outputs
fh = plotStaticSystemOutputComparison(yExpStat, ySimStat, "Exp", "Sim Nominal");
fh.Name = "Sim Outputs / Setpoints";

for iT = 1:length(usedTendons)
    fh = plotStaticSystemOutputComparison( ...
        yExpStat, ySimStat, "Exp", "Sim Nominal", ...
        "plotOverTension", true, "setPointTensions", uSP(iT,:));
    fh.Name = sprintf("Sim Outputs / Tensions T%d", iT);
end

%% Stop script if identification is not enabled
if SIM_ONLY
    return;
end


%% Prepare Identification
%% Define Model parameters

% NLP opti object
opti = casadi.Opti;

calibrationOnly = true;

[IDSystemNLP, IDVars, pVectors] = getParamIDMBSys( ...
    opti, true, calibrationOnly, IDSystemSym, LcOffsetSimDyn);

paramVecNLP = pVectors.NLPVar;


%% Define DEL constraints and system outputs

qDyn_NLP = opti.variable(MBSys.nDoF, nSteps+1);
uDyn_NLP = IDVars.uScale.NLPVar * tensionsExpDyn.';
%uDyn_NLP = tensionsExpDyn.';

% Compute first output time step index to skip time segments at the start
[~, kStart] = min(abs(tout-tStartYError));

% Indices at which the output error is computed

% All (possible) steps
yStepIndices = kStart:nSteps-1;

% Reduced nr. of steps; seems to worsen convergence (maybe due to
% yStepIndices noise in the data)
%yStepIndices = round(linspace(kStart, nSteps-1, nSteps*2/3));


disp("Assembling DEL constraints and system outputs...")
assert(kStart > 1);

qDot0_NLP = opti.variable(MBSys.nDoF);

[c_DEL, yNLPDyn] = prepareDELVarsParamID( ...
    IDSystemNLP, MBSim.simPars, ...
    qDyn_NLP, uDyn_NLP, h, qDot0_NLP, paramVecNLP, yStepIndices, ...
    "useFunctionMap", true);

% Add DEL dynamics constraint
opti.subject_to(c_DEL == 0);


%% Define statics equilibrium constraints and system outputs

nSetpoints = size(uSP,2);
qStat_NLP = opti.variable(MBSys.nDoF, nSetpoints);
uStat_NLP = IDVars.uScale.NLPVar * uSP;

disp("Assembling equilibrium constraints and system outputs...")

[c_Equ, yNLPStat] = prepareStaticEquVarsParamID(IDSystemNLP, MBSim.simPars, ...
    qStat_NLP, uStat_NLP, paramVecNLP, ...
    "useFunctionMap", true);

% Add equilibrium equations as constraint
opti.subject_to(c_Equ == 0);


%% Cost function

disp("Assembling cost function...")

% Output weights
wYDyn  = opti.parameter(5);
wYStat = opti.parameter(3);

% Regulation weights
wR = opti.parameter(1);

% Use the separate cable displacement calibration factors for positive and
% negative measured displacements, as in the static identification.
yExpDynLc = yExpDyn.Lc(:,yStepIndices);
idx_D_tdP = find(yExpDynLc >= 0);
idx_D_tdN = find(yExpDynLc < 0);
idx_S_tdP = find(yExpStat.Lc >= 0);
idx_S_tdN = find(yExpStat.Lc < 0);

yLcOffsetDyn_NLP_P = IDVars.LcScaleP.NLPVar .* (yNLPDyn.Lc - IDVars.LcOffset.NLPVar);
yLcOffsetDyn_NLP_N = IDVars.LcScaleN.NLPVar .* (yNLPDyn.Lc - IDVars.LcOffset.NLPVar);
yLcOffsetStat_NLP_P = IDVars.LcScaleP.NLPVar .* (yNLPStat.Lc - IDVars.LcOffset.NLPVar);
yLcOffsetStat_NLP_N = IDVars.LcScaleN.NLPVar .* (yNLPStat.Lc - IDVars.LcOffset.NLPVar);

% Cost dynamic part
fDyn = ...
    wYDyn(1) * sumsqr(yNLPDyn.IMUGyr1-squeeze(yExpDyn.IMUGyr(:,1,yStepIndices))) ...
    + wYDyn(2) * sumsqr(yNLPDyn.IMUGyr2-squeeze(yExpDyn.IMUGyr(:,2,yStepIndices))) ...
    + wYDyn(3) * sumsqr(yNLPDyn.IMUAcc1-squeeze(yExpDyn.IMUAcc(:,1,yStepIndices))) ...
    + wYDyn(4) * sumsqr(yNLPDyn.IMUAcc2-squeeze(yExpDyn.IMUAcc(:,2,yStepIndices))) ...
    + wYDyn(5) * sumsqr(yLcOffsetDyn_NLP_P(idx_D_tdP)-yExpDynLc(idx_D_tdP)) ...
    + wYDyn(5) * sumsqr(yLcOffsetDyn_NLP_N(idx_D_tdN)-yExpDynLc(idx_D_tdN));

% Cost static part
fStat = ...
    wYStat(1) * sumsqr(yNLPStat.IMUAcc1-squeeze(yExpStat.Acc(:,1,:))) ...
    + wYStat(2) * sumsqr(yNLPStat.IMUAcc2-squeeze(yExpStat.Acc(:,2,:))) ...
    + wYStat(3) * sumsqr(yLcOffsetStat_NLP_P(idx_S_tdP)-yExpStat.Lc(idx_S_tdP)) ...
    + wYStat(3) * sumsqr(yLcOffsetStat_NLP_N(idx_S_tdN)-yExpStat.Lc(idx_S_tdN));


% Regulation term
fR = wR * sumsqr( pVectors.fr .* (paramVecNLP - pVectors.iv));

opti.minimize(fDyn + fStat + fR);


%% Set initial values and bounds

% Model parameters
IDVarFields = fieldnames(IDVars);
for iVar = 1:length(IDVarFields)
    varStruct = IDVars.(IDVarFields{iVar});

    opti.set_initial(varStruct.NLPVar, varStruct.iv);
    opti.subject_to(varStruct.NLPVar(:) >= varStruct.lb(:));
    opti.subject_to(varStruct.NLPVar(:) <= varStruct.ub(:));
end

% Zero initial velocity
opti.subject_to(qDot0_NLP == 0);

% Restrict initial deformation (small bending, no torsion)
q0_ub = MBSys.setLinkDeformations(...
    repmat(+[0.5,0.5,0,0,0,1].', [1, links(1).nSeg]), 1);
q0_lb = MBSys.setLinkDeformations(...
    repmat(-[0.5,0.5,0,0,0,1].', [1, links(1).nSeg]), 1);

opti.subject_to(qDyn_NLP(:,1) >= q0_lb);
opti.subject_to(qDyn_NLP(:,1) <= q0_ub);

opti.set_initial(qDyn_NLP, MBSimIG.simRes.q);
opti.set_initial(qStat_NLP, qSimStat);


%%
% Weights
% Roughly normalize scales of all outputs
sYDAcc = 1/max(abs(yExpDyn.IMUAcc(:)));
sYDGyr = 1/max(abs(yExpDyn.IMUGyr(:)));
sYDLc  = 1/max(abs(yExpDyn.Lc(:)));
sVecD = [sYDGyr,sYDGyr,sYDAcc,sYDAcc,sYDLc];

sYSAcc = 1/max(abs(yExpStat.Acc(:)));
sYSLc  = 1/max(abs(yExpStat.Lc(:)));
sVecS = [sYSAcc,sYSAcc,sYSLc];

opti.set_value(wYDyn, [1,1,1,1, 1000].*sVecD*1e0);
opti.set_value(wYStat, [1,1,1000].*sVecS*50);
opti.set_value(wR, 0.01);


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
    ySolDIMUGyr1 = opti.debug.value(yNLPDyn.IMUGyr1);
    ySolDIMUGyr2 = opti.debug.value(yNLPDyn.IMUGyr2);
    ySolDIMUAcc1 = opti.debug.value(yNLPDyn.IMUAcc1);
    ySolDIMUAcc2 = opti.debug.value(yNLPDyn.IMUAcc2);
    ySolDLcP     = opti.debug.value(yLcOffsetDyn_NLP_P);
    ySolDLcN     = opti.debug.value(yLcOffsetDyn_NLP_N);

    ySolSIMUAcc1 = opti.debug.value(yNLPStat.IMUAcc1);
    ySolSIMUAcc2 = opti.debug.value(yNLPStat.IMUAcc2);
    ySolSLcP     = opti.debug.value(yLcOffsetStat_NLP_P);
    ySolSLcN     = opti.debug.value(yLcOffsetStat_NLP_N);
else
    solLcOffset = sol.value(IDVars.LcOffset.NLPVar);
    ySolDIMUGyr1 = sol.value(yNLPDyn.IMUGyr1);
    ySolDIMUGyr2 = sol.value(yNLPDyn.IMUGyr2);
    ySolDIMUAcc1 = sol.value(yNLPDyn.IMUAcc1);
    ySolDIMUAcc2 = sol.value(yNLPDyn.IMUAcc2);
    ySolDLcP     = sol.value(yLcOffsetDyn_NLP_P);
    ySolDLcN     = sol.value(yLcOffsetDyn_NLP_N);

    ySolSIMUAcc1 = sol.value(yNLPStat.IMUAcc1);
    ySolSIMUAcc2 = sol.value(yNLPStat.IMUAcc2);
    ySolSLcP     = sol.value(yLcOffsetStat_NLP_P);
    ySolSLcN     = sol.value(yLcOffsetStat_NLP_N);
end

ySolDLc = ySolDLcP;
ySolDLc(idx_D_tdN) = ySolDLcN(idx_D_tdN);

ySolSLc = ySolSLcP;
ySolSLc(idx_S_tdN) = ySolSLcN(idx_S_tdN);


% Assign to output struct
yOptDyn = struct();
yOptDyn.yAll = [];
yOptDyn.IMUAcc = cat(2, reshape(ySolDIMUAcc1, 3, 1, []), reshape(ySolDIMUAcc2, 3, 1, []));
yOptDyn.IMUGyr = cat(2, reshape(ySolDIMUGyr1, 3, 1, []), reshape(ySolDIMUGyr2, 3, 1, []));
yOptDyn.Lc     = ySolDLc;
yOptDyn.tout   = tout(yStepIndices);

yOptStat = struct();
yOptStat.Acc = cat(2, reshape(ySolSIMUAcc1, 3, 1, []), reshape(ySolSIMUAcc2, 3, 1, []));
yOptStat.Lc  = ySolSLc;

% plotSystemOutputs(yOpt, "Opt");
% plotSystemOutputComparison(ySim, yOpt);

plotSystemOutputComparison(yExpDyn, yOptDyn, "Exp", "Opt");

fh = plotStaticSystemOutputComparison(yExpStat, yOptStat, "Exp", "Opt");
fh.Name = "Opt Outputs / Setpoints";

for iT = 1:length(usedTendons)
    fh = plotStaticSystemOutputComparison( ...
        yExpStat, yOptStat, "Exp", "Opt", ...
        "plotOverTension", true, "setPointTensions", uSP(iT,:));
    fh.Name = sprintf("Opt Outputs / Tensions T%d", iT);
end



%% Get solution trajectory

if GET_DEBUG_VALUE
    qSol = opti.debug.value(qDyn_NLP);
else
    qSol = sol.value(qDyn_NLP);
end

qDotSol = diff(qSol, 1, 2) / h;
qDotSol_full = [qDotSol, nan(MBSys.nDoF,1)];

MBSimCasadi = MBSim;
MBSimCasadi.Name = "Optimization";
MBSimCasadi.simRes = getSimResFromStateTrajectory(MBSim.MBSys, tout, qSol, qDotSol_full);

%MBSimCasadi.plotAll;
MBSimCasadi.animateSimResults;


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

MBSimVal = MBSimIG;
MBSimVal.MBSys = MBSysOpt;

MBSimVal.simPars.q0 = qSol(:,1);

% Start integration
MBSimVal = MBSimVal.simulateSystem;

% Plotting
%MBSimVal.plotAll;

% Animate results
%MBSimVal.animateSimResults("figureName", "AnimVI");

% Compute system outputs
disp("Computing simulation outputs...")
[yVal, ~] = computeSystemOutputsSim(MBSimVal, IMUDefOpt, cableDefOpt, ...
    "useTendonLengthOffset", false);

% Apply identified cable displacement calibration.
yValLcRel = yVal.Lc - solLcOffset;
yValLcP = paramsRel.LcScaleP .* yValLcRel;
yValLcN = paramsRel.LcScaleN .* yValLcRel;
yVal.Lc = yValLcP;
yVal.Lc(yValLcRel < 0) = yValLcN(yValLcRel < 0);

% Plot outputs
%fhs = plotSystemOutputs(ySim, "Sim");

% Comparison
fhs = plotSystemOutputComparison(yExpDyn, yVal, "Exp", "Opt/Val");


%% Validation simulation statics with identified parameters

[~, yValS, ~] = computeSetPointEqulibria(MBSimVal, uSP, IMUDefOpt, cableDefOpt);

yValSLcRel = yValS.Lc - solLcOffset;
yValSLcP = paramsRel.LcScaleP .* yValSLcRel;
yValSLcN = paramsRel.LcScaleN .* yValSLcRel;
yValS.Lc = yValSLcP;
yValS.Lc(yValSLcRel < 0) = yValSLcN(yValSLcRel < 0);

fh = plotStaticSystemOutputComparison(yExpStat, yValS, "Exp", "Opt/Val");
fh.Name = "Val Outputs / Setpoints";

fh = plotStaticSystemOutputComparison( ...
    yExpStat, yValS, "Exp", "Opt/Val", ...
    "plotOverTension", true, "setPointTensions", uSP(1,:));
fh.Name = "Val Outputs / Tensions";


%% Save identified system parameters

saveFileName = sprintf("IDParams_combined_%s_nSeg_%d", string(datetime, 'yyMMdd_HHmm'), links(1).nSeg);

disp("Saving results...")
fprintf("Filename: %s\n", saveFileName);

save(fullfile(saveFolder,saveFileName), ...
    "MBSysOpt", "IMUDefOpt", "cableDefOpt", ...
    "paramsRel", "links", "dataFileNameDyn", "dataFileNameStat");


%% End script
disp("Finished.")

