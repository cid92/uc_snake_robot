function [ transMatrix ] = transl(x,y,z)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    transMatrix = eye(4);
    transMatrix(:,4) = [x;y;z;1];
end

