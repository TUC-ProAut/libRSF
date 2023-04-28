function [] = writeYAML(File, Struct)
%WRITEYAML Summary of this function goes here
%   Detailed explanation goes here

Struct = convertStringsToChar(Struct);
yaml.WriteYaml(File, Struct, 0);

end

function outputStruct = convertStringsToChar(inputStruct)
    % Create a new struct to store the output
    outputStruct = struct();
    
    % Get the fieldnames of the input struct
    fields = fieldnames(inputStruct);
    
    % Iterate through each field
    for i = 1:numel(fields)
        fieldName = fields{i};
        fieldValue = inputStruct.(fieldName);
        
        if isstruct(fieldValue)
            % If the field value is a struct, recursively call the function
            outputStruct.(fieldName) = convertStringsToChar(fieldValue);
        elseif iscell(fieldValue)
            % If the field value is a cell, iterate through each element
            % and recursively call the function on each element
            outputCell = cell(size(fieldValue));
            for j = 1:numel(fieldValue)
                cellValue = fieldValue{j};
                if ischar(cellValue)
                    % If the cell value is a string, convert it to a char array
                    outputCell{j} = char(cellValue);
                elseif isstruct(cellValue)
                    % If the cell value is a struct, recursively call the function
                    outputCell{j} = convertStringsToChar(cellValue);
                elseif iscell(cellValue)
                    % If the cell value is another cell, recursively call the function
                    % on each element of the inner cell
                    outputCell{j} = convertStringsToChar(cellValue);
                else
                    % Otherwise, keep the value unchanged
                    outputCell{j} = cellValue;
                end
            end
            outputStruct.(fieldName) = outputCell;
        elseif isstring(fieldValue)
            % If the field value is a string, convert it to a char array
            outputStruct.(fieldName) = char(fieldValue);
        else
            % Otherwise, keep the value unchanged
            outputStruct.(fieldName) = fieldValue;
        end
    end
end