function yAcc = applyAccelerometerCalibration(yAcc, calibFile)
    %% Apply Accelerometer Calibration to Measurement Data
    %
    % Calibration model: aTrue = M * (aMeas - b)
    % Maximilian Herrmann
    % Chair of Automatic Control
    % TUM School of Engineering and Design
    % Technical University of Munich
    arguments (Input)
        % Accelerometer data
        % Dimensions 3 x nIMUs x nValues
        yAcc        (3,:,:) double

        % Full path to the mat file containing calibration data;
        % must contain gain matrix M and bias vector b for all IMUs
        calibFile   (1,1) string
    end
    arguments (Output)
        yAcc (3,:,:) double
    end

    nIMUs = size(yAcc,2);

    % Load calibration data from file
    calibData = load(calibFile);

    % Check matching dimensions
    assert(size(calibData.M,3) == nIMUs);
    assert(size(calibData.b,2) == nIMUs);

    % Apply calibration
    for iIMU = 1:nIMUs
        yAcc(:,iIMU,:) = calibData.M(:,:,iIMU)*...
            (squeeze(yAcc(:,iIMU,:)) - calibData.b(:,iIMU));
    end
end