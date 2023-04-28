function [] = writeData(Data, Path, MeasurementFilename, GroundTruthFilename)
%WRITEDATA Summary of this function goes here
%   Detailed explanation goes here

% parse sensor data struct to cell
[MeasurementCell, GTCell] = libRSF.parseToCell(Data);
% write to file
writecell_fast(MeasurementCell, fullfile(Path, MeasurementFilename));

% write ground truth
writecell_fast(GTCell, fullfile(Path, GroundTruthFilename));

end

%% 10 times faster than matlabs writecell
function [] = writecell_fast(Cell, Filename)
    fileID = fopen(Filename,'w');
    Format = ['%s ' repmat('%.14g ',1,size(Cell,2)-1) '\n'];
    for n = 1:size(Cell,1)
        fprintf(fileID, Format, Cell{n,:});
    end
    fclose(fileID);
end