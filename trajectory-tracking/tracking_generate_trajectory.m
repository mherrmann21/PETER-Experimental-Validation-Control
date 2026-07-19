%% Generate Trajectories for PETER Traj. Tracking Experiment
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Script settings

% Compute more accurate initial guess based on static positions?
% False = use zero initial guess
COMPUTE_IG = 1;

% Where to save the generated trajectory
saveFolder = fullfile(getRootFolder, "data", "trajectories");


%% Define System
usedTendons = [1,2,3];
links = systemDef_PETER_nominal_reduced("nSeg", 8, ...
    "usedTendons", usedTendons, "d", ones(6,1)*25.0e-3);

MBSim = MBSimulation(links, "displayInfo", false);
MBSysSym = MBSystemSym(links);


%% Define OCP

OCP = OCPDefinition;
OCP.MBSys = MBSystemSym(links);

OCP.q0    = zeros(MBSim.MBSys.nDoF,1);
OCP.qDot0 = zeros(MBSim.MBSys.nDoF,1); % Initial velocity
OCP.qDotF = zeros(MBSim.MBSys.nDoF,1); % Final velocity

OCP.u0 = [];
OCP.uMin = [3,3,3,1.75];
OCP.uMin = OCP.uMin(usedTendons);
OCP.uMax = [];

% End time, sample time
OCP.h = 2^-7;
OCP.tF = 3.75;

OCP.simPars = MBSim.simPars;

OCP.wRC = [
    5e-1   % Norm u
    1e-3   % Norm u_dot
    10e-2  % Norm u_ddot
    50e-1  % Norm q_ddot
    1e7    % TCP error / Running tracking error
    ]*1e-3;
OCP.iRC = logical(OCP.wRC);

% No final time cost term
OCP.iFC = zeros(3,1);

OCP.addTCPFinalTimeConstraint = false;

OCP.useSplineInputs = true;
OCP.inputSplineOrder = 3;
OCP.nInputSplinePoints = 25;

% NLP object / solver options
OCP.nlpOpts.ipopt.warm_start_init_point = 'no';
OCP.nlpOpts.expand = false;
OCP.nlpOpts.ipopt.acceptable_tol = 1e-5;
OCP.nlpOpts.ipopt.tol = 1e-6; % Default 1e-8

%% Visualize reference configuration and target position

[~, vis] = MBSim.visualizeSystemRefConf();
coordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));
drawWorkspace(OCP.workSpaceDef, "createFigure", false);


%% Compute initial condition: equilibrium configuration with pretensions

disp("Computing initial equilibrium configuration...");
opts = optimoptions('fsolve', ...
    'Display','none', ...
    'Algorithm','trust-region', ...
    'FunctionTolerance', 1e-10, ...
    'OptimalityTolerance', 1e-10);

[qEqu, fval, exitflag, output] = fsolve( ...
    @(q) computeStaticResiduum_mex(MBSim.MBSys, MBSim.simPars, q, OCP.uMin), ...
    OCP.q0, opts ...
    );

OCP.q0 = qEqu;

% TCP position of the initial configuration
gEqu = MBSim.MBSys.computeFwdKin(qEqu);
g_TCP = gEqu(:,:,MBSim.MBSys.indexTCPFrame)*MBSim.MBSys.g_B_TCP;
x_TCP_des = g_TCP(1:3, 4);

%% Generate desired TCP trajectory

% Pre and post actuation times
OCP.tPreAct  = 2^-2;
OCP.tPostAct = 2^-2;

% Specify TCP trajectory waypoints
nSegPoints = 4;
az = deg2rad([zeros(1,nSegPoints), linspace(120/(nSegPoints), 120, nSegPoints)]);
el = deg2rad([linspace(60,0, nSegPoints), zeros(1,nSegPoints)]+30);

offset = 0.12;
r = ones(size(az)) * 0.69 - offset;

[x,y,z] = sph2cart(az,el,r);
z = z + offset;

% Generate smooth trajectory from waypoints
xpts = [x;y;z];

xpts(:,1) = x_TCP_des;

tpts = linspace(OCP.tPreAct, OCP.tF-OCP.tPostAct, length(az));
x_TCP_traj_dyn = minjerkpolytraj( xpts, tpts, length(OCP.tout), ...
    "TimeAllocation", true, "TimeWeight", 130);

% Resample trajectory for OCP time vector
tVecInterp = linspace(OCP.tPreAct, OCP.tF - OCP.tPostAct, size(x_TCP_traj_dyn, 2));
if OCP.tPreAct
    x_TCP_traj_dyn = [x_TCP_traj_dyn(:,1), x_TCP_traj_dyn];
    tVecInterp = [0, tVecInterp];
end
if OCP.tPostAct
    x_TCP_traj_dyn = [x_TCP_traj_dyn, x_TCP_traj_dyn(:,end)];
    tVecInterp = [tVecInterp, OCP.tF];
end
x_TCP_traj_dyn = interp1( ...
    tVecInterp, ...
    x_TCP_traj_dyn.', ...
    OCP.tout, "spline", "extrap" ...
    ).';

% Assign trajectory to OCP object
OCP.x_TCP_traj = x_TCP_traj_dyn;

% Assign waypoints to OCP object (used only for the initial guess)
nWpts = size(xpts,2);
OCP.x_TCP_waypoints = xpts(:, [1, round(nWpts/2), nWpts]);
OCP.x_TCP_timepoints = tpts([1, round(nWpts/2), nWpts]);
if OCP.tPreAct
    OCP.x_TCP_waypoints = [OCP.x_TCP_waypoints(:,1), OCP.x_TCP_waypoints];
    OCP.x_TCP_timepoints = [0, OCP.x_TCP_timepoints];
end
if OCP.tPostAct
    OCP.x_TCP_waypoints = [OCP.x_TCP_waypoints, OCP.x_TCP_waypoints(:,end)];
    OCP.x_TCP_timepoints = [OCP.x_TCP_timepoints, OCP.tF];
end


% 3D Plot
init3Dplot("Name", "Generated TCP Trajectory", "NumberTitle", "off");
plot3(xpts(1,:), xpts(2,:), xpts(3,:), "-o");
hold on;
plot3(x_TCP_traj_dyn(1,:), x_TCP_traj_dyn(2,:), x_TCP_traj_dyn(3,:));
legend("Waypoints", "Generated Trajectory")


% Plot over time
[x_TCP_traj_dyn_dt, x_TCP_traj_dyn_ddt] = diff2ndOrder(x_TCP_traj_dyn, OCP.h);
if OCP.Name == ""
    figPrefix = "";
else
    figPrefix = strcat(OCP.Name, ": ");
end
fhs(1) = figure("Name", strcat(figPrefix, " TCP Trajectory Desired"), "NumberTitle", "off");
tiledlayout("vertical");

nexttile;
plot(OCP.tout, x_TCP_traj_dyn);
grid on;
ylabel("$x_{TCP}$ in m", "Interpreter", "latex")
xlabel("$t$ in s", "Interpreter", "latex")
legend("$x$", "$y$", "$z$", "interpreter", "latex");
title("Desired TCP trajectory");

nexttile;
plot(OCP.tout, x_TCP_traj_dyn_dt);
grid on;
ylabel("$\dot{x}_{TCP}$ in m/s", "Interpreter", "latex")
xlabel("$t$ in s", "Interpreter", "latex")

nexttile;
plot(OCP.tout, x_TCP_traj_dyn_ddt);
grid on;
ylabel("$\ddot{x}_{TCP}$ in m/s/s", "Interpreter", "latex")
xlabel("$t$ in s", "Interpreter", "latex")

% 3D visualization
fhs(2) = figure("Name", strcat(figPrefix, "TCP Trajectory Vis"), "NumberTitle", "off");
init3Dplot("createFigure",false);
MBSim.visualizeSystemConfig(OCP.q0, "createFigure", false);
plot3(x_TCP_traj_dyn(1,:), x_TCP_traj_dyn(2,:), x_TCP_traj_dyn(3,:), "-o");

xlim([ ...
    min([0, min(x_TCP_traj_dyn(1,:))])-0.1, ...
    max([0, max(x_TCP_traj_dyn(1,:))])+0.1, ...
    ]);
ylim([ ...
    min([0, min(x_TCP_traj_dyn(2,:))])-0.1, ...
    max([0, max(x_TCP_traj_dyn(2,:))])+0.1, ...
    ]);
zlim([ ...
    min([0, min(x_TCP_traj_dyn(3,:))]), ...
    max([0.8, max(x_TCP_traj_dyn(3,:))])+0.1, ...
    ]);



%% Compute Initial Guess

ANIMATE_IG = true;
invDynMethod = "DEL";

if COMPUTE_IG
    [q_init, qd_init, u_initO, MBSimIG, qOptStatic, uOptStatic] = OCPComputeInitialGuess_InvDyn( ...
        MBSim, OCP, "invDynMethod", "DEL", "createDebugPlots", true);

    if invDynMethod == "DEL"
        u_initO(:,end) = u_initO(:,end-1);
    end

    % Slightly smooth inputs to remove peaks in derivatives
    u_init = movmean(u_initO, 50, 2);

    MBSim.visualizeSystemConfig(qOptStatic, "figureName", "Vis. Optimal Static Config.");
    drawWorkspace(OCP.workSpaceDef, "createFigure", false);

    % Animate results
    if 0%ANIMATE_IG
        fig = init3Dplot('Name', "Animation Initial Guess");%, "WindowStyle","normal");
        coordSysSE3(SE3Matrix(eye(3), OCP.x_TCP_F));
        drawWorkspace(OCP.workSpaceDef, "createFigure", false);
        MBSimIG.animateSimResults("figure", fig);
    end

else
    q_init = repmat(OCP.q0, [1, OCP.nSteps+1]);
    u_init = repmat(OCP.uMin, [1, OCP.nSteps+1]);
end
plotOCPqu(OCP, q_init, u_init, "figureName", "Initial Guess", "plotDerivatives", true);

if OCP.useSplineInputs
    % Compute control points for initial guess
    B = OCP.getInputSplineBasisMatrix;
    u_init_z =  (B \ u_init.').';

    % Plot initial guess fit
    figure("Name", "Initial Guess B-Spline Fit");
    tiledlayout("vertical");
    nexttile;
    plot(OCP.tout, u_init, "-.x", "DisplayName", "Original Data");
    hold on;
    plot(OCP.tout, B*u_init_z.', "--o", "DisplayName", "Fitted Spline");
    grid on;
    colororder(lines(MBSim.MBSys.nInputs));
    legend;
    title("Spline Fit");

    nexttile;
    plot(OCP.tout, abs(u_init.'-B*u_init_z.'));
    grid on;
    title("Fit Error");
else
    u_init_z = u_init;
end

%% Define DEL OCP Solver

OCP_DEL = OCP;
OCP_DEL.Name ="VI";
OCP_DEL.discretization = OCPIntegratorVI;
OCP_DEL.discretization.aTrapez = 0.5;

OCP_DEL = OCP_DEL.initSolver("useCasadiStepFunctions", true, "showDebugPlots",true);

% Plot constraint residuals of the initial guess
OCP_DEL.plotConstraintResiduals(q_init, u_init_z, "figureName", "Constr. Res. IG");

% Initial guess objective components
if ~OCP_DEL.useSplineInputs
    disp("Objective function components initial guess:")
    disp(cellfun( @(x) full(x), OCP_DEL.constrDef.Fun_fRComp.call({quMat2XVec(q_init, u_init), OCP.x_TCP_F, OCP_DEL.wRC}) ))
end

%% Solve OCP
% with weights and x_TCP trajectory specified in OCP object

[q_sol, u_sol_z, sol, stats] = OCP_DEL.solve(q_init, u_init_z);

% Plot solution data
OCP_DEL.plotConstraintResiduals(q_sol, u_sol_z, "figureName", "Constr. Res. Solution");

if OCP_DEL.useSplineInputs
    u_sol = (B*u_sol_z.').';
else
    u_sol = u_sol_z;
end

plotOCPqu(OCP_DEL, q_sol, u_sol, "plotDerivatives", true, "FDOrder", 2);

if ~OCP_DEL.useSplineInputs
    disp("Objective function components solution:")
    disp(cellfun( @(x) full(x), OCP_DEL.constrDef.Fun_fRComp.call({quMat2XVec(q_sol, u_sol), OCP.x_TCP_F, OCP.wRC}) ))
end

fh = plotOCPTCPTraj(MBSim, OCP, q_sol);

%% Post-process etc.

disp('Post processing...')
gTCPDes = SE3Matrix(eye(3), OCP_DEL.x_TCP_F);

q_dot_Sol = diff(q_sol, 1, 2) / OCP_DEL.h;
q_dot_Sol_full = [q_dot_Sol, nan(MBSysSym.nDoF,1)];

MBSimCasadi = MBSim;
MBSimCasadi.Name = "Optimization";
MBSimCasadi.simRes = getSimResFromStateTrajectory(MBSim.MBSys, OCP_DEL.tout, q_sol, q_dot_Sol_full);

%MBSimCasadi.plotAll;

% Draw snapshots
fig = init3Dplot('Name', "Snapshots Solution", "NumberTitle", "off");%, "WindowStyle","normal");
%coordSysSE3(gTCPDes);
drawWorkspace(OCP_DEL.workSpaceDef, "createFigure", false);
if OCP_DEL.nSteps < 50
    nSnapShots = OCP_DEL.nSteps/2+1;
else
    nSnapShots = 20;
end
MBSimCasadi.drawSnapshots("figure", fig, "nSnapShots",nSnapShots);
TCPTraj = squeeze(MBSimCasadi.simRes.g(1:3,4,end,:));
plot3(TCPTraj(1,:),TCPTraj(2,:),TCPTraj(3,:), '-');
plot3(x_TCP_traj_dyn(1,:), x_TCP_traj_dyn(2,:), x_TCP_traj_dyn(3,:));


% Animate results
fig = init3Dplot('Name', "Animation Solution");%, "WindowStyle","normal");
%coordSysSE3(gTCPDes);
drawWorkspace(OCP.workSpaceDef, "createFigure", false);
plot3(TCPTraj(1,:),TCPTraj(2,:),TCPTraj(3,:), '-');
plot3(x_TCP_traj_dyn(1,:), x_TCP_traj_dyn(2,:), x_TCP_traj_dyn(3,:));
MBSimCasadi.animateSimResults("figure", fig, "saveMovie", false, "fileName","example_optControl_contManip");


%% Save Trajectory

saveFileName = sprintf("trajectory_%s_nSeg_%d", ...
    string(datetime, 'yyMMdd_HHmm'), links(1).nSeg);

tout = OCP.tout;
uPreTension = OCP.uMin;
h = OCP.h;

disp("Saving results...")
save(fullfile(saveFolder,saveFileName), ...
    "MBSim", "MBSimCasadi", "q_sol", "u_sol", "sol", "stats", ...
    "q_init", "u_init_z", ...
    "OCP", "tout", "uPreTension", "h");


%% End script
disp("Finished.")
