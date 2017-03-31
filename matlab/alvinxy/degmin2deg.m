function [degFloat] = degmin2deg(degInt,minFloat)

% convert decimal degrees  to integer degrees and decimal minutes
% i.e., ddd.dddd -> dd mm.mmmm
%
% bbing 17.06.04
%

% check that degress are integer
if round(degInt)~=degInt
    fprintf('Input degrees need to be integer \n');
    degFloat = -1;
    return 
end

degFloat = degInt+minFloat/60;
