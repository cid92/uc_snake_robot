function [ transMatrix ] = trotz( angle )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
 transMatrix = [cos(angle) 0 sin(angle) 0;...
     0 1 0 0;...
     -sin(angle) 0 cos(angle) 0;...
     0 0 0 1];
end
