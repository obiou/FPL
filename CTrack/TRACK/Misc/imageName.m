%
% @see discoverImageNameFormat.m
%
function filename = imageName(imageInfo,value)
name=[imageInfo.Path imageInfo.Prefix '%0' num2str(imageInfo.nbVal) 'd' imageInfo.Suffix];
filename = sprintf(name,value);
