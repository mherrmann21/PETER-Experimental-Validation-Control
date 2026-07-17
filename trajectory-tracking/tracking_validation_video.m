%% Validate trajectory tracking with video overlay of dynamic motion over video
%
% Maximilian Herrmann
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich

clear
close all


%% Script settings

% Save as movie file?
saveMovie = true;
saveFileNamePrefixVideo = "exp_";

% Save individual snapshot images? (only if useSubPlots = false)
saveSnapshots = true;

% True = Save frames in subplots in one figure instead of creating individual figures for all frames
useSubPlots = false;

% File with the generated trajectory data
trajDataFolder   = fullfile(getRepositoryRootFolder, "data", "trajectories");
trajDataFileName = "trajectory_251218_1610_nSeg_8";

% Plot save folder
plotSaveDir = fullfile(getRepositoryRootFolder, "results", "tracking");

%%% Image/video data
% fileNameCalibImage: High-Res image that is used to estimate the camera/robot pose
% fileNameVideo:      Actual video file name
%
% tEnd      End time of the video
% tStart    At which time the video is started, i.e., time of the first
%           video frame used
% tOffset   Offset of the video time to TCP trajectory time
%           * should be larger than tStart
%           * if the reference trajectory is ahead of the video trajectory,
%             increase tOffset
imageFolder = fullfile(getRepositoryRootFolder, "data", "experiments", "photos", "Trajectory Tracking 251218_1639");
fileNameCalibImage = "251218_1639_tracking_refImage.JPG";
fileNameVideo      = "251218_1639_tracking_video.MTS";
tOffset = 11.96;
tStart  = 11.4;
tEnd    = 17.3;


%% Load camera calibration

% Camera calibration file (generated from MATALB computer vision toolbox)
cameraCalibFile = fullfile(getRepositoryRootFolder, "data", "calibration", ...
    "camera-params-sony-50mm-16-9-251212-1520.mat");

data = load(cameraCalibFile);
intrinsics = data.cameraParams;


%% Load trajectory  data

trajData = load(fullfile(trajDataFolder, trajDataFileName));


%% Get Camera Pose from Calibration Image

[pose_0, camIntrinsics, im] = getCameraPose( ...
    fullfile(imageFolder, fileNameCalibImage), intrinsics, "phiGlobal", 5);

% Draw trajectory
imPts_xTCPRef = world2img(trajData.OCP.x_TCP_traj.', pose_0, camIntrinsics);
plot(imPts_xTCPRef(:,1),imPts_xTCPRef(:,2), "-", ...
    "LineWidth", 1.5, "Color", [0,1,0,0.75]);


%% Load video frames
% Note: We directly access the individual frames instead of using the
% vR.CurrentTime functionality since the latter seems to produce
% inconsistent results

vR = VideoReader(fullfile(imageFolder, fileNameVideo));
frameTimesAll = linspace(0, vR.Duration, vR.NumFrames);

[~, iStartFrame] = min(abs(frameTimesAll - tStart));
[~, iEndFrame] = min(abs(frameTimesAll - tEnd));

videoFrames = read(vR, [iStartFrame, iEndFrame]);
frameTimes = frameTimesAll(iStartFrame:iEndFrame);

%% Create individual overlaid frames

% Times of the frames to show
snapShotTimes = linspace(tStart+0.8, tStart + trajData.tout(end) + 0.1, 6);

if useSubPlots
    figure("NumberTitle", "off", "Name", "videoFrames");
    tiledlayout("flow", "TileSpacing", "tight");
end

for iFrm = 1:length(snapShotTimes)
    [~, frameIndex] = min(abs(frameTimes - snapShotTimes(iFrm)));
    frame = videoFrames(:,:,:,frameIndex);

    % Time of the current frame in video time
    curTimeVid = frameTimes(frameIndex);

    % Time of the current frame in trajectory time
    curTimeTraj = curTimeVid - tOffset;

    fprintf("Frame index: %d, currentTimeVideo: %f, currentTimeTraj: %f\n", frameIndex, curTimeVid, curTimeTraj)

    if useSubPlots
        nexttile;
    else
        fh = figure("NumberTitle", "off", ...
            "Name", sprintf("frame %d t=%.3f", iFrm, curTimeTraj));
    end

    drawVideoFrame(frame, im, intrinsics, pose_0, curTimeVid, ...
        trajData.OCP.x_TCP_traj, trajData.tout, tOffset);

    if useSubPlots
        title(sprintf("Current Time = %.3f s", curTimeTraj));
    end

    % Save image
    if saveSnapshots && ~useSubPlots
        box off;
        saveFileName = fullfile(plotSaveDir, fh.Name + ".jpg");
        exportgraphics(fh, saveFileName, "Resolution", 300, "Padding",0);
    end
end


%% Create overlaid video

% Struct where the frame data is stored
animFrame = struct("cdata", [], "colormap", []);

fh = figure("Name", "OverlayVideo");
for iFrm = 1:length(frameTimes)% hasFrame(vR) && isvalid(fh) && vR.CurrentTime < tEnd
    %frame = readFrame(vR);
    frame = videoFrames(:,:,:,iFrm);

    drawVideoFrame(frame, im, intrinsics, pose_0, frameTimes(iFrm), ...
        trajData.OCP.x_TCP_traj, trajData.tout, tOffset);

    if saveMovie
        animFrame(end+1) = getframe(fh);
    else
        drawnow;
    end
end

%% Write to video
% (code from MATLAB docs)

if saveMovie
    disp('Saving as Video...')

    % Remove first frame, which is an empty frame from the struct
    % definition
    animFrame = animFrame(2:end);

    % Check whether all video frames have the same size
    firstDimension = size(animFrame(1).cdata);
    sizesEqual = true;
    for iFrame = 1:length(animFrame)
        if ~all(size(animFrame(iFrame).cdata) == firstDimension)
            sizesEqual = false;
            warning("Could not save video to file since the video frame size " + ...
                "has changed during the animation. " + ...
                "The size of the animation figure must be kept constant.");
            break;
        end
    end

    % Write actual video
    videoSavePath = fullfile(plotSaveDir, saveFileNamePrefixVideo + trajDataFileName);
    if sizesEqual
        v = VideoWriter(videoSavePath, 'MPEG-4');
        v.Quality = 100;
        v.FrameRate = vR.FrameRate;
        open(v);
        for iFrame = 1:length(animFrame)
            writeVideo(v,animFrame(iFrame));
        end
        close(v);
    end
end


%% End script
disp("Finished.")

%% Local functions

function [pose_0, camIntrinsics, im] = getCameraPose(imPath, intrinsics, opts)
    %% Get the camera pose from a single image from an AruCo marker
    arguments
        % Full image paths (vector)
        imPath          (1,1) string

        % Camera calibration parameters
        intrinsics      (1,1) cameraParameters

        % Global rotation of the robot base around the z-axis
        opts.phiGlobal  (1,1) double = 0;
    end
    imOrientation = 0;

    % Load and undistort image
    im = imread(imPath);
    [im, camIntrinsics] = undistortImage(im,intrinsics);

    % Detect ArUco marker and estimate robot pose
    [pose_0, im] = estimateRobotPoseFromPhoto(im,camIntrinsics, "phiGlobal", opts.phiGlobal);

    % Rotate image if needed
    if imOrientation
        imPlot = imrotate(im, -90);
    else
        imPlot = im;
    end

    % Plot image
    figure("Name", "Calibration Image");
    imshow(imPlot);
    hold on;
end

function drawVideoFrame(frame, imCalib, intrinsics, pose_0, currentTime, x_TCP_traj, tout, tOffset)
    %% Draw a complete video frame with overlay
    arguments
        % Image to draw on
        frame           (:,:,3) uint8

        % Calibration image for pose estimation; needed for undistorting
        % the video frame (with possibly lower resolution)
        imCalib         (:,:,3) uint8

        % Camera calibration parameters corresponding to imCalib
        intrinsics      (1,1) cameraParameters

        % Camera pose
        pose_0          (1,1) rigidtform3d

        % Time of the current frame in the video
        currentTime     (1,1) double

        % TCP trajectory over time (absolute values)
        x_TCP_traj      (3,:) double

        % Time values of the TCP trajectory
        tout            (:,1) double

        % Offset between video time and trajectory time
        tOffset         (1,1) double
    end

    cols = lines(3);

    useLargeImageSize = false;
    if useLargeImageSize
        % For large output images
        trajLineWidth = 2.5;
        trajMarkerSize = 9;
        trajAlpha = 0.65;
    else
        % For small images in the thesis
        trajLineWidth = 7.5;
        trajMarkerSize = 25;
        trajAlpha = 0.8;
    end


    % Upscale frame to calibration image and undistort with calibration
    % image parameters
    frame = imresize(frame, size(imCalib, [1,2]));
    [frame, camIntrinsics] = undistortImage(frame, intrinsics);

    hold off; % Clear previous figure contents to save memory
    imshow(frame)
    %title(sprintf("Current Time = %.3f sec", currentTime))
    hold on;

    % Draw trajectory
    imPts_xTCPRef = world2img(x_TCP_traj.', pose_0, camIntrinsics);
    plot( imPts_xTCPRef(:,1), imPts_xTCPRef(:,2), "-", ...
        "LineWidth", trajLineWidth, "Color", [0,1,0,trajAlpha]);

    % Draw current TCP target point
    if currentTime < tOffset
        imPts_xTCPRef_cur = imPts_xTCPRef(1,:);
    elseif currentTime > tOffset + tout(end)
        imPts_xTCPRef_cur = imPts_xTCPRef(end,:);
    else
        imPts_xTCPRef_cur = interp1( tout + tOffset, imPts_xTCPRef, currentTime );
    end
    % plot(imPts_xTCPRef_cur(1), imPts_xTCPRef_cur(2), "o", ...
    %     "LineWidth", 4.5, "MarkerSize", 9, "Color", [0,1,0]);
    plot(imPts_xTCPRef_cur(1), imPts_xTCPRef_cur(2), "o", ...
        "LineWidth", trajLineWidth, "MarkerSize", trajMarkerSize, ...
        "Color", cols(2,:));

    % Draw coordinate frame at the base
    imPts_x0 = world2img(zeros(1,3), pose_0, camIntrinsics);
    imPts_xF = world2img(eye(3)*0.03, pose_0, camIntrinsics);
    plot([imPts_x0(1), imPts_xF(1,1)], [imPts_x0(2), imPts_xF(1,2)], "-r", ...
        "LineWidth", 2);
    plot([imPts_x0(1), imPts_xF(2,1)], [imPts_x0(2), imPts_xF(2,2)], "-g", ...
        "LineWidth", 2);
    plot([imPts_x0(1), imPts_xF(3,1)], [imPts_x0(2), imPts_xF(3,2)], "-b", ...
        "LineWidth", 2);
end