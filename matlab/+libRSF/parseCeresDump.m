function [Dump] = parseCeresDump(Path)
%ANALYZECERESDUMP Summary of this function goes here
%   Detailed explanation goes here

if nargin < 1
    RelativePath = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    Path = convertCharsToStrings(RelativePath) + filesep + "build" + filesep + "examples" + filesep;
end

% we delete files in this script, so we use the trash for safety
RecycleState = recycle;
recycle('on');

% name of the matlab scripts created by ceres
ScriptBaseName = "ceres_solver_iteration_";

DumpExists = false;
IterationExist = true;
IterationNumber = 1;
while IterationExist
    ScriptNameFull = Path + ScriptBaseName + num2str(IterationNumber, '%03i')+ ".m";
    
    if isfile(ScriptNameFull)
        % run script to parse the data
        run(ScriptNameFull);
        
        % copy result into struct array
        Dump(IterationNumber) = ans;
        
        % delete matlab file
        delete(ScriptNameFull);
        
        % there is a dump from ceres
        DumpExists = true;
    else
        % no more script --> end here
        IterationExist = false;
    end  
    
    IterationNumber =  IterationNumber + 1;
end

% restore recycle state
recycle(RecycleState);

%% preprocess
if DumpExists == true
    x_cum = cumsum([Dump.x],2)';
    cost = 0.5*sum([Dump.b].^2,1)';
    
    for n=1:numel(Dump)
        Dump(n).x_cum = x_cum(n,:);
        Dump(n).cost = cost(n);
    end
else
    Dump = [];
end

end


