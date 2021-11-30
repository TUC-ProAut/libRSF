function [Result] = wrapCeres(Data, Config, ExecutableFile, DataSetName, RewriteData, RewriteYAML)
%WRAPCERES Summary of this function goes here
%   Detailed explanation goes here

% if not specified, we write everything again
if nargin < 5
  RewriteData = true;
end
if nargin < 6
  RewriteYAML = true;
end

%% find right build folder
% folders relative to the script path
RelativePath = fileparts(mfilename('fullpath'));
PossibleDirs = {[filesep '..' filesep '..' filesep 'build'],...
                [filesep '..' filesep '..' filesep '..' filesep 'build']};

BuildDir = [];
for n = 1:numel(PossibleDirs)
    if isfolder([RelativePath PossibleDirs{n}])
      BuildDir = [RelativePath PossibleDirs{n}];
    end
end
if isempty(BuildDir)
    error('Did not find build directory for libRSF binary!');
end


%% find right binary subfolder
PathToBinaryExample = [filesep 'examples' filesep];
PathToBinaryApplication = [filesep 'applications' filesep];

if startsWith(ExecutableFile,'App') || startsWith(ExecutableFile,'I') 
    PathToBinary = [BuildDir PathToBinaryApplication];
else
    PathToBinary = [BuildDir PathToBinaryExample];
end

%% set filenames
DataFile = [DataSetName '_Input.txt'];
GTFile = [DataSetName '_GT.txt'];
OutputFile = [DataSetName '_Output.txt'];
ConfigFile = [DataSetName '_Config.yaml'];
RefFile = [DataSetName '_Reference.yaml'];

%% write config
if RewriteYAML
    if isfield(Config, 'config')
        yaml.WriteYaml([PathToBinary ConfigFile], Config, 0);
    end
end

%% write data to file
if RewriteData
    disp('Starting to write the sensor data to file...')
    % parse sensor data struct to cell
    [MeasurementCell, GTCell] = libRSF.parseToCell(Data);
    % write to file
    writecell_fast(MeasurementCell,[PathToBinary DataFile]);
    
    % write ground truth
    writecell_fast(GTCell,[PathToBinary GTFile]);
end

%% save reference Data
libRSF.writeYamlReference(Data, [PathToBinary RefFile]);

%% call ceres
% check for application
if ~isfile([PathToBinary ExecutableFile])
    error(['Could not find file: ' PathToBinary ExecutableFile]);
end

% construct call
if isfield(Config, 'Custom')
    if startsWith(ExecutableFile,'IV')  || startsWith(ExecutableFile,'ICRA')
        % Legacy applications without config file
        CeresCall = ['.' filesep ExecutableFile ' ' DataFile ' ' OutputFile ' ' Config.Custom];
    else
        CeresCall = ['.' filesep ExecutableFile ' ' ConfigFile ' ' DataFile ' ' OutputFile ' ' Config.Custom];
    end
else
    CeresCall = ['.' filesep ExecutableFile ' ' ConfigFile ' ' DataFile ' ' OutputFile];
end

% change path and call binary
OldPath = cd(PathToBinary);
    disp('Calling the libRSF binary...');
    tic;
        BinaryResult = system(CeresCall);
    Runtime = toc;
cd(OldPath);

if BinaryResult ~= 0
    warning('Ceres binary failure!');
end

%% parse results
ResultCell = readcell([PathToBinary OutputFile], 'FileType', 'text', 'NumHeaderLines', 0, 'Delimiter', ' ');
Result = libRSF.parseFromCell(ResultCell);
Result.Runtime = Runtime;

%% save time difference
if isfield(Data, 'Info')
    Result.StartTime = Data.Info.StartTime;
else
    Result.StartTime = 0;
end

%% load ceres dump if available
Result.Dump = libRSF.parseCeresDump(PathToBinary);

%% add empty solver summary if missing
if ~isfield(Result, 'SolverSummary')
    Result.SolverSummary = struct([]);
end
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
