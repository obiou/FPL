%
% @see discoverImageNameFormat.m
%
function [I,fileName] = imageLoad( imageInfo, value )
fileName = imageName( imageInfo, value );
I = double( imread( fileName ) );
