function [imgid, kappa_gt, omega_gt, phi_gt, x_gps, x_gt, y_gps, y_gt, z_gps, z_gt] = loadGroundTruthAGL()
% Adapted from Zurich loadGroundTruthAGL script, but in a function form

    %% Initialize variables.
    filename = '../data/Log Files/GroundTruthAGL.csv';
    delimiter = ',';
    startRow = 2;

    %% Format string for each line of text:
    %   column1: double (%f)
    %	column2: double (%f)
    %   column3: double (%f)
    %	column4: double (%f)
    %   column5: double (%f)
    %	column6: double (%f)
    %   column7: double (%f)
    %	column8: double (%f)
    %   column9: double (%f)
    %	column10: double (%f)
    % For more information, see the TEXTSCAN documentation.
    formatSpec = '%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

    %% Open the text file.
    fileID = fopen(filename,'r');

    %% Read columns of data according to format string.
    % This call is based on the structure of the file used to generate this
    % code. If an error occurs for a different file, try regenerating the code
    % from the Import Tool.
    dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, ...
        'EmptyValue' ,NaN,'HeaderLines' ,startRow-1, 'ReturnOnError', false);

    %% Close the text file.
    fclose(fileID);

    %% Post processing for unimportable data.
    % No unimportable data rules were applied during the import, so no post
    % processing code is included. To generate code which works for
    % unimportable data, select unimportable cells in a file and regenerate the
    % script.

    %% Allocate imported array to column variable names
    imgid = dataArray{:, 1};
    x_gt = dataArray{:, 2};
    y_gt = dataArray{:, 3};
    z_gt = dataArray{:, 4};
    omega_gt = dataArray{:, 5};
    phi_gt = dataArray{:, 6};
    kappa_gt = dataArray{:, 7};
    x_gps = dataArray{:, 8};
    y_gps = dataArray{:, 9};
    z_gps = dataArray{:, 10};
end