function [array1,array2] = table2array(Table)
%FUNCTIONS Summary of this function goes here
%   Detailed explanation goes here
    array1 = [];
    array2 = [];
    for i = 1:length(Table.name)   
        if char(Table.name(i)) == 'CC2640r2 Passive 1'
            array1 = [array1; Table.angle(i)];
        elseif char(Table.name(i)) == 'CC2640r2 Passive 2'
            array2 = [array2; Table.angle(i)];
        end
    end
end

