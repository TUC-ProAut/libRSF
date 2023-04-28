function [Result] = wrapCeres(Data, Config, ExecutableFile, DatasetName, RewriteData, RewriteYAML)
%WRAPCERES Summary of this function goes here
%   Detailed explanation goes here

% if not specified, we write everything again
if nargin < 5
  RewriteData = true;
end
if nargin < 6
  RewriteYAML = true;
end

if ischar(ExecutableFile)
    ExecutableFile = convertCharsToStrings(ExecutableFile);
end
if ischar(DatasetName)
    DatasetName = convertCharsToStrings(DatasetName);
end

%% find right build folder
% folders relative to the script path
RelativePath = fileparts(mfilename('fullpath'));
PossibleDirs = [fullfile(".." , ".." , "build"),...
                fullfile("..", ".." , ".." , "build")];

BuildDir = [];
for n = 1:numel(PossibleDirs)
    if isfolder(fullfile(RelativePath, PossibleDirs(n)))
      BuildDir = fullfile(RelativePath, PossibleDirs(n));
    end
end
if isempty(BuildDir)
    error("Did not find build directory for libRSF binary!");
end

%% find right binary subfolder
PathToBinaryExample = "examples";
PathToBinaryApplication = "applications";

if startsWith(ExecutableFile,"App") || startsWith(ExecutableFile,"ICRA") || startsWith(ExecutableFile,"IV") 
    PathToBinary = fullfile(BuildDir, PathToBinaryApplication);
else
    PathToBinary = fullfile(BuildDir, PathToBinaryExample);
end

%% handle parallel execution
Task = getCurrentTask();
if isempty(Task)
    WorkerID = "";
else
    WorkerID = "_" + num2str(Task.ID);
end

%% set filenames
DataFile = DatasetName + WorkerID + "_Input.txt";
GTFile = DatasetName + WorkerID + "_GT.txt";
OutputFile = DatasetName + WorkerID + "_Output.txt";
ConfigFile = DatasetName + WorkerID + "_Config.yaml";
RefFile = DatasetName + WorkerID + "_Reference.yaml";

%% write config
if RewriteYAML
    if isfield(Config, "config")
        libRSF.writeYAML(fullfile(PathToBinary, ConfigFile), Config);
    end
end

%% write data to file
if RewriteData
    disp("Starting to write the sensor data to file...")
    libRSF.writeData(Data, PathToBinary, DataFile, GTFile);
end

%% save reference Data
libRSF.writeYamlReference(Data, fullfile(PathToBinary, RefFile));

%% call ceres
% check for application
if ~isfile(fullfile(PathToBinary, ExecutableFile))
    error("Could not find file: " + fullfile(PathToBinary, ExecutableFile));
end

% construct call
if isfield(Config, "Custom")
    if startsWith(ExecutableFile,"IV")  || startsWith(ExecutableFile, "ICRA")
        % Legacy applications without config file
        CeresCall = fullfile('.' , ExecutableFile) + " " + DataFile + " " + OutputFile + " " + Config.Custom;
    else
        CeresCall = fullfile('.' , ExecutableFile) + " " + ConfigFile + " " + DataFile + " " + OutputFile + " " + Config.Custom;
    end
else
    CeresCall = fullfile('.' , ExecutableFile)+ " " + ConfigFile + " " + DataFile + " " + OutputFile;
end

% change path and call binary
OldPath = cd(PathToBinary);
    disp("Calling the libRSF binary...");
    tic;
        BinaryResult = system(CeresCall);
    Runtime = toc;
cd(OldPath);

if BinaryResult ~= 0
    warning("Ceres binary failure!");
end

%% parse results
ResultCell = readcell(fullfile(PathToBinary, OutputFile), 'FileType', 'text', 'NumHeaderLines', 0, 'Delimiter', ' ');
Result = libRSF.parseFromCell(ResultCell);
Result.Runtime = Runtime;

%% save time difference
if isfield(Data, "Info")
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
