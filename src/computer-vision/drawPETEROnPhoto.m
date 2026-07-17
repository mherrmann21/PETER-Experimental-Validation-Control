function drawPETEROnPhoto(MBCSys,g, pose_0, camIntrinsics, imOrientation, opts)
    %% Visualize PETER on a photo
    arguments (Input)
        MBCSys (1,1) MBControlSystem
        % Backbone configuration
        g (4,4,:) double {mustBeSE3MatrixArray}

        pose_0

        camIntrinsics

        imOrientation (1,1) double

        % Line width multiplicator
        opts.lineWidth (1,1) double = 1;
    end

    %% Draw spacer disks

    tendonDef = MBCSys.tendonDef;

    %%% Compute disk circle points in local XY plane
    % Vector in homogeneous notation
    rDisks = 22.5e-3;
    th = linspace(0,2*pi,50);
    diskPointsLocal = [
        rDisks * cos(th);
        rDisks * sin(th);
        zeros(1, length(th));
        ones(1, length(th))
        ];

    % Get backbone configuration at spacer disk points
    sInput = linspace(0,max(tendonDef.sDisks),size(g,3));
    s_sd_l = tendonDef.sDisks-1e-3;
    s_sd_l(s_sd_l<0) = 0;
    s_sd_u = tendonDef.sDisks+1.5e-3;
    g_sd_l   = interpSE3(g, sInput, s_sd_l);
    g_sd_u   = interpSE3(g, sInput, s_sd_u);

    imagePoints_sd_l = zeros(length(th), 2, tendonDef.nDisks);
    imagePoints_sd_u = zeros(length(th), 2, tendonDef.nDisks);

    cols = lines(3);

    for iDisk = 1:tendonDef.nDisks
        % Transform to absolute points
        diskPointsAbs_l = g_sd_l(:,:,iDisk) * diskPointsLocal;
        diskPointsAbs_u = g_sd_u(:,:,iDisk) * diskPointsLocal;
        x_sd_l = diskPointsAbs_l(1:3,:).';
        x_sd_u = diskPointsAbs_u(1:3,:).';
        imagePoints_sd_l(:,:,iDisk) = world2img(x_sd_l, pose_0, camIntrinsics);
        imagePoints_sd_u(:,:,iDisk) = world2img(x_sd_u, pose_0, camIntrinsics);


        %%% TODO: Rotate image data, if needed

        % Plot
        patch( ...
            'XData',imagePoints_sd_l(:,1, iDisk),'YData',imagePoints_sd_l(:, 2, iDisk), ...
            'FaceColor', cols(2,:), 'FaceAlpha', 0.1, "EdgeAlpha", 0.75, ...
            'EdgeColor', cols(2,:), 'LineWidth', 1.0*opts.lineWidth ...
            );
        patch( ...
            'XData',imagePoints_sd_u(:,1, iDisk),'YData',imagePoints_sd_u(:, 2, iDisk), ...
            'FaceColor', cols(2,:), 'FaceAlpha', 0.1, "EdgeAlpha", 0.75, ...
            'EdgeColor', cols(2,:), 'LineWidth', 1.0*opts.lineWidth ...
            );
    end

    % Additionally get points of spacer disk centers
    imagePoints_sd_c = world2img(squeeze(g_sd_l(1:3,4,:)).', pose_0, camIntrinsics);
    plot(imagePoints_sd_c(:,1),imagePoints_sd_c(:,2), ".", ...
        "MarkerSize",10, "Color", "g");


    %% Draw robot backbone

    % Interpolate backbone configuration
    lInterp = 0.01;
    sInput_norm = linspace(0,1,size(g,3)); % normalized
    sQuery_norm = 0:lInterp:1;
    gInterp_bb = interpSE3(g, sInput_norm, sQuery_norm);
    x_bb = squeeze(gInterp_bb(1:3,4,:)).';

    imagePoints_bb = world2img(x_bb, pose_0, camIntrinsics);

    % Rotate data if needed
    if imOrientation
        imagePoints_bb = [size(im,1) - imagePoints_bb(:,2), imagePoints_bb(:,1)];
    end

    % Plot backbone
    plot(imagePoints_bb(:,1),imagePoints_bb(:,2), "-", ...
        "LineWidth", 1.5*opts.lineWidth, "Color", [0,1,0,0.75]);


    %% Draw IMUs

    IMUDef = MBCSys.IMUDef;

    % Get backbone configuration at IMU positions
    s_IMU = IMUDef.s;
    s_sd_l = s_IMU-1.0e-3;
    s_sd_u = s_IMU+1.5e-3;
    g_bb_IMU   = interpSE3(g, sInput, s_IMU);
    g_bb_IMU_l = interpSE3(g, sInput, s_sd_l);
    g_bb_IMU_u = interpSE3(g, sInput, s_sd_u);


    %%% Compute IMU corner points in local IMU frame
    % Vector in homogeneous notation
    wIMU = 27.5e-3;
    hIMU = 19.5e-3;
    IMUPointsLocal = [
        -wIMU, -wIMU, +wIMU, +wIMU, -wIMU
        -hIMU, +hIMU, +hIMU, -hIMU, -hIMU
        zeros(1, 5);
        ones(1, 5)*2;
        ]/2;

    for iIMU = 1:length(IMUDef.s)
        % Get IMU frames
        g_IMU   = g_bb_IMU(:,:,iIMU) * IMUDef.g_rel(:,:,iIMU);
        g_IMU_l = g_bb_IMU_l(:,:,iIMU) * IMUDef.g_rel(:,:,iIMU);
        g_IMU_u = g_bb_IMU_u(:,:,iIMU) * IMUDef.g_rel(:,:,iIMU);

        % Draw line from backbone center to IMU
        x_bb_IMU = [g_bb_IMU(1:3,4,iIMU), g_IMU(1:3,4)].';
        imagePoints_IMU = world2img(x_bb_IMU, pose_0, camIntrinsics);

        plot(imagePoints_IMU(:,1), imagePoints_IMU(:,2), ".-", ...
            "LineWidth", 1.1, "MarkerSize", 12, "Color", cols(3,:));


        % Transform corners to absolute points
        IMUPointsAbs_l = g_IMU_l * IMUPointsLocal;
        IMUPointsAbs_u = g_IMU_u * IMUPointsLocal;
        x_IMU_l = IMUPointsAbs_l(1:3,:).';
        x_IMU_u = IMUPointsAbs_u(1:3,:).';
        imagePoints_IMU_l = world2img(x_IMU_l, pose_0, camIntrinsics);
        imagePoints_IMU_u = world2img(x_IMU_u, pose_0, camIntrinsics);


        %%% TODO: Rotate image data, if needed

        % Plot
        patch( ...
            'XData',imagePoints_IMU_l(:,1),'YData',imagePoints_IMU_l(:, 2), ...
            'FaceColor', cols(2,:), 'FaceAlpha', 0.1, "EdgeAlpha", 0.75, ...
            'EdgeColor', cols(2,:), 'LineWidth', 1.0 ...
            );
        patch( ...
            'XData',imagePoints_IMU_u(:,1),'YData',imagePoints_IMU_u(:, 2), ...
            'FaceColor', cols(2,:), 'FaceAlpha', 0.1, "EdgeAlpha", 0.75, ...
            'EdgeColor', cols(2,:), 'LineWidth', 1.0 ...
            );
    end
end