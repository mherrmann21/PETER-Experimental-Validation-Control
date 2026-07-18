function [pose_0,im] = estimateRobotPoseFromPhoto(im,camIntrinsics, opts)
    %% Estimate the pose of PETER in a photo using AruCo markers
    arguments (Input)
        % Image data (already undistorted)
        im              (:,:,3)
        camIntrinsics

        % Global rotation of the robot base around the z-axis
        opts.phiGlobal (1,1) double = 0;
    end

    arguments (Output)
        % Pose of the robot base w.r.t. camera
        pose_0

        % Input image with added poses of the ArUco marker and backbone
        im
    end

        %% Detect ArUco marker
        markerSize = 80 * 1e-3; % m
        markerFamily = "DICT_4X4_50";
        [markerIDs,~, markerPoses] = readArucoMarker(im, markerFamily ,camIntrinsics, markerSize);


        %% Overlay estimated marker poses

        % Origin and axes vectors for the object coordinate system
        worldPoints = [0 0 0; markerSize/2 0 0; 0 markerSize/2 0; 0 0 markerSize/2];

        for i = 1:length(markerPoses)
            % Get image coordinates for axes
            imagePoints = world2img(worldPoints,markerPoses(i),camIntrinsics);

            axesPoints = [imagePoints(1,:) imagePoints(2,:);
                imagePoints(1,:) imagePoints(3,:);
                imagePoints(1,:) imagePoints(4,:)];
            % Draw colored axes
            im = insertShape(im, "Line", axesPoints, ...
                Color = ["red","green","blue"], LineWidth=10);
        end

        %% Compute and visualize backbone base position

        zeroIndex = find(markerIDs == 0);

        pose_m = markerPoses(zeroIndex(1));

        % Marker center -> bb origin
        x_m_0 = [
            -50
            -50-20
            -3.15
            ]*1e-3;

        % Constant rotation applied to the backbone origin to compensate
        %  rotation of the modal base frame
        phi_global = opts.phiGlobal;
        R_bb = [
            cosd(phi_global), -sind(phi_global), 0;
            sind(phi_global),  cosd(phi_global), 0;
            0           , 0            , 1];

        g_m_0 = SE3Matrix(eye(3), x_m_0);
        g_rot = SE3Matrix(R_bb, zeros(3,1));
        pose_0 = rigidtform3d(pose_m.A*g_m_0*g_rot);

        imagePoints = world2img(worldPoints, pose_0, camIntrinsics);

        axesPoints = [imagePoints(1,:) imagePoints(2,:);
            imagePoints(1,:) imagePoints(3,:);
            imagePoints(1,:) imagePoints(4,:)];

        % Draw colored axes
        im = insertShape(im, "Line", axesPoints, ...
            Color = ["red","green","blue"], LineWidth=10);

end
