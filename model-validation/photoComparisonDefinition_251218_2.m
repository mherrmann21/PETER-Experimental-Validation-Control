function compSetPoints = photoComparisonDefinition_251218_2()
    %% Define images and setpoints for visual photo/model comparison

    uPreTension = 3; % Common pretension
    imFolder = fullfile(getRootFolder, "data", "experiments", "photos", "Statics Identification 251218-2 SPs 6-9-12");

    % Set up definition struct
    %   .imOrientation      0 = landscape, 1 = portrait
    %   .imFolder           (folder)
    %   .u                  tendon tensions
    compSetPoints = struct();
    
    uStat = linspace(0,16,12);

    % Setpoints T1
    compSetPoints(1).imNames = [
        "_DSC1604.JPG"
        "_DSC1606.JPG"
        ];
    compSetPoints(end).imFolder = imFolder;
    compSetPoints(end).imOrientation = [0 0];
    compSetPoints(end).u = [uStat(6),0,0].' + uPreTension;

    compSetPoints(end+1).imNames = [
        "_DSC1607.JPG"
        "_DSC1609.JPG"
        ];
    compSetPoints(end).imFolder = imFolder;
    compSetPoints(end).imOrientation = [0 0];
    compSetPoints(end).u = [uStat(9),0,0].' + uPreTension;

    compSetPoints(end+1).imNames = [
        "_DSC1610.JPG"
        "_DSC1612.JPG"
        ];
    compSetPoints(end).imFolder = imFolder;
    compSetPoints(end).imOrientation = [0,0];
    compSetPoints(end).u = [uStat(12),0,0].' + uPreTension;


    % Setpoints T2
    compSetPoints(end+1).imNames = [
        "_DSC1616.JPG"
        "_DSC1618.JPG"
        ];
    compSetPoints(end).imFolder = imFolder;
    compSetPoints(end).imOrientation = [0 0];
    compSetPoints(end).u = [0,uStat(6),0].' + uPreTension;

    compSetPoints(end+1).imNames = [
        "_DSC1619.JPG"
        "_DSC1621.JPG"
        ];
    compSetPoints(end).imFolder = imFolder;
    compSetPoints(end).imOrientation = [0,0];
    compSetPoints(end).u = [0,uStat(9),0].' + uPreTension;

    compSetPoints(end+1).imNames = [
        "_DSC1623.JPG"
        "_DSC1626.JPG"
        ];
    compSetPoints(end).imFolder = imFolder;
    compSetPoints(end).imOrientation = [0,0];
    compSetPoints(end).u = [0,uStat(12),0].' + uPreTension;


    % Setpoints T3
    compSetPoints(end+1).imNames = [
        "_DSC1630.JPG"
        "_DSC1631.JPG"
        ];
    compSetPoints(end).imOrientation = [0 0];
    compSetPoints(end).imFolder = imFolder;
    compSetPoints(end).u = [0,0,uStat(6)].' + uPreTension;

    compSetPoints(end+1).imNames = [
        "_DSC1634.JPG"
        "_DSC1635.JPG"
        ];
    compSetPoints(end).imFolder = imFolder;
    compSetPoints(end).imOrientation = [0 0];
    compSetPoints(end).u = [0,0,uStat(9)].' + uPreTension;


    compSetPoints(end+1).imNames = [
        "_DSC1638.JPG"
        "_DSC1639.JPG"
        ];
    compSetPoints(end).imFolder = imFolder;
    compSetPoints(end).imOrientation = [0,0];
    compSetPoints(end).u = [0,0,uStat(12)].' + uPreTension;

end