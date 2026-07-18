%% Startup File: Add file paths and check important dependencies

% Absolute file path of this script
funPath = mfilename("fullpath");

% Get the directory of the script = repository root path
rootPath = fileparts(funPath);

% Add important paths
addpath(genpath(fullfile(rootPath, "src")));
addpath(genpath(fullfile(rootPath, "model-validation")));
addpath(genpath(fullfile(rootPath, "trajectory-tracking")));

% Check if ELARA toolbox is available on the path
assert(exist("MBSimulation", "file"), ...
    "ELARA toolbox functions are not found on the MATLAB path. " + ...
    "Make sure the toolbox is installed correctly.")

% Check if CasADi is installed correctly
assert(exist("casadi.MX", "class"), ...
    "CasADi is not found on the MATLAB path. " + ...
    "Make sure it is installed correctly.")
