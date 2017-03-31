function [degInt,minFloat] = deg2degmin(degFloat)

% convert decimal degrees  to integer degrees and decimal minutes
% i.e., ddd.dddd -> dd mm.mmmm
%
% bbing 17.06.04
%

degFloat = abs(degFloat);
degInt = floor(degFloat);
minFloat = (degFloat-degInt)*60;
