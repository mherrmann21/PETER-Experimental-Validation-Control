%% Combine several sets of static setpoint data

%% Script settings

dataFolder = fullfile(getRootFolder, "data", "experiments", "processed");

dataFileNames = [
    "251218_1440_id_data_static_tendon_1_setpoints.mat"
    "251218_1440_id_data_static_tendon_2_setpoints.mat"
    "251218_1440_id_data_static_tendon_3_setpoints.mat"
    ];

% Select setpoints
%idxSetpoints = 1:12;
idxSetpoints = 1:22;


%% Combine data

u = zeros(4,0);
yAcc = zeros(3,2,0);
yLt = zeros(4,0);

for iFile = 1:length(dataFileNames)
    expData = load(fullfile(dataFolder, dataFileNames(iFile)));

    u = [u, expData.u(:,idxSetpoints)];
    yAcc = cat(3, yAcc, expData.yAcc(:,:,idxSetpoints));
    yLt = [yLt, expData.yLt(:,idxSetpoints)];
end

%% Save combined data
saveFileName = replace(dataFileNames(1), ".mat", "_combined.mat");
save(fullfile(dataFolder, saveFileName), "yLt", "yAcc", "u");

%% End script
disp("Finished.");
