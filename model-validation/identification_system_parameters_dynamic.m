%% Identify system parameters for PETER (tendon-actuated continuum manipulator)
% based on data from dynamic motions
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Script settings

% Only simulate system with nominal parameters?
SIM_ONLY = false;

% Runs the identification on the initial guess data with fixed parameters
% to validate the NLP implementation w.r.t. to the numeric implementation
TEST_SYM_FRAMEWORK = false;

% Experimental data
dataFolder   = fullfile(getRepositoryRootFolder, "data", "experiments", "raw");
dataFileName = "251105_1732_exp_data.mat";

% IMU calibration data
accCalibFile = fullfile(getRepositoryRootFolder, "data", "calibration", "IMUCalib_260717_1307");

% Output folder
saveFolder = fullfile(getRepositoryRootFolder, "data", "identification");

% System and model settings
usedTendons = 1;
nSeg = 5;

% Dynamic data parameters
tStartOffset = 1;
tStartYError = 0.02; % Start time when the output error is being computed
tYShift = 0;
tEnd = 3.3;

% Time step
h = 2^-7;


%% Define Nominal System

links = systemDef_PETER_nominal_reduced("nSeg", nSeg, "usedTendons", usedTendons);
MBSim = MBSimulation(links, "displayInfo", false);

[IMUDef, tendonDef] = definePETEROutputs(links);

MBSys = MBSim.MBSys;
MBSysSym = MBSystemSym(links);


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


%% Load experiment data

% Simulation time vector (has length nSteps + 1)
nSteps = round( MBSim.simPars.tEnd / h );
tout = (0:h:h*nSteps)';

[tensionsExp, yExp] = getExperimentData(fullfile(dataFolder, dataFileName), tout, ...
    "tStartOffset", tStartOffset, "tYShift", tYShift, ...
    "usedTendons", usedTendons, ...
    "fCTendons", 25, "fCOutput", 27, ...
    "useReferenceTensions", false);

% Apply accelerometer calibration
yExp.IMUAcc = applyAccelerometerCalibration(yExp.IMUAcc, accCalibFile);

% Plot system outputs
plotSystemOutputs(yExp, "Exp");


%% Initial guess: Simulate with measured system inputs

MBSimIG = MBSim;
MBSimIG.Name = "IG Nominal";

MBSimIG.simPars.uSampleTimes  = tout;
MBSimIG.simPars.uSampleValues = tensionsExp.';

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
[ySim, LtOffsetSim]  = computeSystemOutputsSim(MBSimIG, IMUDef, tendonDef);

% Plot outputs
%fhs = plotSystemOutputs(ySim, "Sim");

% Comparison
plotSystemOutputComparison(yExp, ySim, "Exp", "IG Nominal");

if SIM_ONLY
    return;
end



%% Prepare Identification
%% Define Model parameters

% NLP opti object
opti = casadi.Opti;

IDSystemNum = struct;
IDSystemNum.MBSys    = MBSys;
IDSystemNum.IMUDef   = IMUDef;
IDSystemNum.tendonDef = tendonDef;

IDSystemSym = IDSystemNum;
IDSystemSym.MBSys = MBSysSym;

[IDSystemNLP, IDVars, pVectors] = getParamIDMBSys( ...
    opti, true, false, IDSystemSym, LtOffsetSim);

paramVecNLP = pVectors.NLPVar;

%% Define DEL constraints and system outputs

q_NLP = opti.variable(MBSys.nDoF, nSteps+1);
u_NLP = IDVars.uScale.NLPVar * tensionsExp.';

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

[c_DEL, yNLP] = prepareDELVarsParamID( IDSystemNLP, MBSim.simPars,...
    q_NLP, u_NLP, h, qDot0_NLP, paramVecNLP, yStepIndices, ...
    "useFunctionMap", true);

% Add DEL dynamics constraint
opti.subject_to(c_DEL == 0);


%% Cost function

disp("Assembling cost function...")

% Output weights
wY = opti.parameter(5);

% Regulation weights
wR = opti.parameter(1);

if TEST_SYM_FRAMEWORK
    yExp = ySim;
end

% Use the separate tendon displacement calibration factors for positive and
% negative measured displacements, as in the static identification.
yExpLt = yExp.Lt(:,yStepIndices);
idx_tdP = find(yExpLt >= 0);
idx_tdN = find(yExpLt < 0);

yLtOffset_NLP_P = IDVars.LtScaleP.NLPVar .* (yNLP.Lt - IDVars.LtOffset.NLPVar);
yLtOffset_NLP_N = IDVars.LtScaleN.NLPVar .* (yNLP.Lt - IDVars.LtOffset.NLPVar);

f = ...
    wY(1) * sumsqr(yNLP.IMUGyr1-squeeze(yExp.IMUGyr(:,1,yStepIndices))) ...
    + wY(2) * sumsqr(yNLP.IMUGyr2-squeeze(yExp.IMUGyr(:,2,yStepIndices))) ...
    + wY(3) * sumsqr(yNLP.IMUAcc1-squeeze(yExp.IMUAcc(:,1,yStepIndices))) ...
    + wY(4) * sumsqr(yNLP.IMUAcc2-squeeze(yExp.IMUAcc(:,2,yStepIndices))) ...
    + wY(5) * sumsqr(yLtOffset_NLP_P(idx_tdP)-yExpLt(idx_tdP)) ...
    + wY(5) * sumsqr(yLtOffset_NLP_N(idx_tdN)-yExpLt(idx_tdN));

% Regulation term
fR = wR * sumsqr(pVectors.fr .* (paramVecNLP - pVectors.iv));

opti.minimize(f + fR);


%% Set initial values and bounds

% Model parameters
IDVarFields = fieldnames(IDVars);
for iVar = 1:length(IDVarFields)
    varStruct = IDVars.(IDVarFields{iVar});

    opti.set_initial(varStruct.NLPVar, varStruct.iv);
    if TEST_SYM_FRAMEWORK
        opti.subject_to(varStruct.NLPVar(:) == varStruct.iv(:));
    else
        opti.subject_to(varStruct.NLPVar(:) >= varStruct.lb(:));
        opti.subject_to(varStruct.NLPVar(:) <= varStruct.ub(:));
    end
end

% Zero initial velocity
opti.subject_to(qDot0_NLP == 0);

% Restrict initial deformation (small bending, no torsion)
q0_ub = MBSys.setLinkDeformations( ...
    repmat(+[0.5,0.5,0,0,0,1].', [1, links(1).nSeg]), 1);
q0_lb = MBSys.setLinkDeformations( ...
    repmat(-[0.5,0.5,0,0,0,1].', [1, links(1).nSeg]), 1);
opti.subject_to(q_NLP(:,1) >= q0_lb);
opti.subject_to(q_NLP(:,1) <= q0_ub);

opti.set_initial(q_NLP, MBSimIG.simRes.q);

%%
% Weights
% Roughly normalize scales of all outputs
sYAcc = 1/max(abs(yExp.IMUAcc(:)));
sYGyr = 1/max(abs(yExp.IMUGyr(:)));
sYLt  = 1/max(abs(yExp.Lt(:)));
sVec = [sYGyr,sYGyr,sYAcc,sYAcc,sYLt];

opti.set_value(wY, [1,1,1e-5,1e-5, 1].*sVec*1e0);
opti.set_value(wR, 0.005);


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
    solLtOffset = opti.debug.value(IDVars.LtOffset.NLPVar);
    ySolIMUGyr1 = opti.debug.value(yNLP.IMUGyr1);
    ySolIMUGyr2 = opti.debug.value(yNLP.IMUGyr2);
    ySolIMUAcc1 = opti.debug.value(yNLP.IMUAcc1);
    ySolIMUAcc2 = opti.debug.value(yNLP.IMUAcc2);
    ySolLtP     = opti.debug.value(yLtOffset_NLP_P);
    ySolLtN     = opti.debug.value(yLtOffset_NLP_N);
else
    solLtOffset = sol.value(IDVars.LtOffset.NLPVar);
    ySolIMUGyr1 = sol.value(yNLP.IMUGyr1);
    ySolIMUGyr2 = sol.value(yNLP.IMUGyr2);
    ySolIMUAcc1 = sol.value(yNLP.IMUAcc1);
    ySolIMUAcc2 = sol.value(yNLP.IMUAcc2);
    ySolLtP     = sol.value(yLtOffset_NLP_P);
    ySolLtN     = sol.value(yLtOffset_NLP_N);
end

ySolLt = ySolLtP;
ySolLt(idx_tdN) = ySolLtN(idx_tdN);


% Assign to output struct
yOpt = struct();
yOpt.yAll = [];
yOpt.IMUAcc = cat(2, reshape(ySolIMUAcc1, 3, 1, []), reshape(ySolIMUAcc2, 3, 1, []));
yOpt.IMUGyr = cat(2, reshape(ySolIMUGyr1, 3, 1, []), reshape(ySolIMUGyr2, 3, 1, []));
yOpt.Lt     = ySolLt;
yOpt.tout   = tout(yStepIndices);

% plotSystemOutputs(yOpt, "Opt");
% plotSystemOutputComparison(ySim, yOpt);
plotSystemOutputComparison(yExp, yOpt, "Exp", "Opt");


%% Get solution trajectory

if GET_DEBUG_VALUE
    qSol = opti.debug.value(q_NLP);
else
    qSol = sol.value(q_NLP);
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

paramsRel.LtOffset = solLtOffset;


%% Get numeric MBSystem with identified parameters

IDSystemOpt = getMBSysParamsFromIDVars(IDSystemNum, IDSystemNLP, sol);

MBSysOpt    = IDSystemOpt.MBSys;
tendonDefOpt = IDSystemOpt.tendonDef;
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
[yVal, ~] = computeSystemOutputsSim(MBSimVal, IMUDefOpt, tendonDefOpt, ...
    "useTendonLengthOffset", false);

% Apply identified tendon displacement calibration.
yValLtRel = yVal.Lt - solLtOffset;
yValLtP = paramsRel.LtScaleP .* yValLtRel;
yValLtN = paramsRel.LtScaleN .* yValLtRel;
yVal.Lt = yValLtP;
yVal.Lt(yValLtRel < 0) = yValLtN(yValLtRel < 0);

% Plot outputs
%fhs = plotSystemOutputs(ySim, "Sim");

% Comparison
fhs = plotSystemOutputComparison(yExp, yVal, "Exp", "Opt/Val");


%% Save identified system parameters

saveFileName = sprintf("IDParams_dyn_%s_nSeg_%d", string(datetime, 'yyMMdd_HHmm'), links(1).nSeg);

disp("Saving results...")
fprintf("Filename: %s\n", saveFileName);

save(fullfile(saveFolder,saveFileName), ...
    "MBSysOpt", "IMUDefOpt", "tendonDefOpt", ...
    "paramsRel", "links", "dataFileName");


%% End script
disp("Finished.")
