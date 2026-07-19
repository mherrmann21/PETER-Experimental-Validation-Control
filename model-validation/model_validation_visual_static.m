%% Visually compare static configurations of PETER to photos
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Script settings

SAVE_PLOTS = 1;

% Plot save folder
plotSaveDir = fullfile(getRootFolder, "results", "validation");

% Camera calibration file (generated with the MATLAB Computer Vision Toolbox)
cameraCalibFile = fullfile(getRootFolder, "data", "calibration", ...
    "camera-params-sony-50mm-3-2-251117-1849.mat");


%% Specify images

compSetPoints = photoComparisonDefinition_251218_2();

%% Load system
usedTendons = [1,2,3];

if 1
    % Nominal system
    links = systemDef_PETER_nominal_reduced("nSeg", 12, "usedTendons", usedTendons);
    MBSim = MBSimulation(links, "displayInfo", false);

    [IMUDef, tendonDef] = definePETEROutputs(links);

    MBSys = MBSim.MBSys;
    MBSysSym = MBSystemSym(links);

else
    % Identified system model
    identifiedModelPath = fullfile(getRootFolder, "data", "identification", "IDParams_static_251127_2045_nSeg_8");

    % Load model from identification
    IDstruct = load(identifiedModelPath);
    links = IDstruct.links;
    MBSim = MBSimulation(links, "displayInfo", false);
    MBSim.MBSys = IDstruct.MBSysOpt;
    MBSys    = IDstruct.MBSysOpt;
    MBSysSym = MBSystemNum2MBSystemSym(MBSys);
    IMUDef = IDstruct.IMUDefOpt;
    tendonDef = IDstruct.tendonDefOpt;
    MBSim.MBSys = MBSys;
end

MBCSys = MBControlSystem(MBSys, IMUDef, tendonDef);


%% Visualization

% MBSim.visualizeSystemRefConf;
% axis tight;
% zlim([0, 0.7]);


%% Simulate setpoints

uSP = [compSetPoints.u];
[qSimStat, ySimStat, ~] = computeSetPointEqulibria(MBSim, uSP, IMUDef, tendonDef);


%% Draw setpoints

for iSP = 1:length(compSetPoints)

    % Get backbone shape
    g_bb = MBSys.computeFwdKin(qSimStat(:,iSP));
    g_bb = cat(3, eye(4), g_bb);

    if SAVE_PLOTS
        % Draw all images in separate figures and save to file
        for iIm = 1:length(compSetPoints(iSP).imNames)
            fprintf("Drawing setpoint %d/%d, image %d/%d...\n", ...
                iSP, length(compSetPoints), iIm, length(compSetPoints(iSP).imNames));


            imPath = fullfile(compSetPoints(iSP).imFolder, compSetPoints(iSP).imNames(iIm));

            fh = drawOverlayImage(MBCSys, g_bb, imPath, ...
                compSetPoints(iSP).imOrientation(iIm), cameraCalibFile, ...
                "phiGlobal", 0, "lineWidth", 1.5);

            fh.Name = sprintf("Setpoint %d image %d", iSP, iIm);
            fh.NumberTitle = "off";

            % Save image
            box off;
            saveFileName = fullfile(plotSaveDir, fh.Name + ".jpg");
            exportgraphics(fh, saveFileName, "Resolution", 300, "Padding",0);
        end
    else
        % Draw multiple images in one figure as subplots
        imPaths = fullfile(compSetPoints(iSP).imFolder, compSetPoints(iSP).imNames);

        fh = drawOverlayImage(MBCSys, g_bb, imPaths, ...
            compSetPoints(iSP).imOrientation, cameraCalibFile, ...
            "phiGlobal", 0);

        fh.Name = sprintf("Setpoint %d", iSP);
        fh.NumberTitle = "off";
    end
end

disp("Finished.");


%% Local functions
function fh = drawOverlayImage(MBCSys, g, imPaths, imOrientation, filePathCameraCalib, opts)
    arguments
        MBCSys (1,1) MBControlSystem

        % Backbone configuration
        g (4,4,:) double {mustBeSE3MatrixArray}

        % Full image paths (vector)
        imPaths (:,1) string

        % Image orientation (0 = landscape, 1 = portrait)
        imOrientation (:,1) double

        % Camera calibration file (generated with the MATLAB Computer Vision Toolbox)
        filePathCameraCalib (1,1) string

        % Global rotation of the robot base around the z-axis
        opts.phiGlobal (1,1) double = 0;

        % Line-width multiplier
        opts.lineWidth (1,1) double = 1;
    end

    %% Load camera calibration

    data = load(filePathCameraCalib);
    intrinsics = data.cameraParams;

    %% Generate images
    fh = figure;
    if length(imPaths) > 1
        tiledlayout("flow", "TileSpacing", "tight", "Padding", "compact");
    end

    for iIm = 1:length(imPaths)
        %% Load image
        im = imread(imPaths(iIm));

        % undistort
        [im, camIntrinsics] = undistortImage(im,intrinsics);


        %% Detect ArUco marker and estimate robot pose
        [pose_0, im] = estimateRobotPoseFromPhoto(im,camIntrinsics, "phiGlobal", opts.phiGlobal);


        %% Plot image
        if length(imPaths) > 1
            nexttile;
        end

        % Rotate image if needed
        if imOrientation(iIm)
            imPlot = imrotate(im, -90);
        else
            imPlot = im;
        end

        imshow(imPlot);
        hold on;


        %% Plot robot
        drawPETEROnPhoto(MBCSys, g, pose_0, camIntrinsics, imOrientation(iIm), ...
            "lineWidth", opts.lineWidth);

    end
end
