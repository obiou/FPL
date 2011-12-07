% DISCOVERIMAGENAMEFORMAT - This function tries to discover the image
% format based on a string of the name of the first image and returns
% a structure that can be used in conjunction with loadImg.m
%
% Input:
%   - imageNameExample: string of image name
%   - endValue (optional): last value of sequence, this can be used
%   to avoid directory scanning or to impose specific end value
%
% Output:
%   - imageInfo: structure containing image information
function imageInfo = discoverImageNameFormat( imageNameExample, endValue )

if nargin==2
  imageInfo.End = endValue;
  bFindEnd = 0;
else
  bFindEnd = 1;
end

% Look for . defining the suffix of the name (required)
dotsInName = strfind( imageNameExample, '.' );

if isempty( dotsInName )
  fprintf( 'ERROR in "discoverImageNameFormat", could not find any dots at the end of name' );
  return
end

lastDot = dotsInName( end );
imageInfo.Suffix = imageNameExample( lastDot:end );

leftOver = imageNameExample( 1:lastDot-1 );

% Look for the path delimiters
if isunix
  dirDelimiters = strfind( leftOver, '/' );
else
  dirDelimiters = strfind( leftOver, '\' );
end

if isempty( dirDelimiters )
  imageInfo.Path = '';
else
  lastDirDelimiter = dirDelimiters( end );
  imageInfo.Path = leftOver( 1:lastDirDelimiter );
  leftOver = leftOver( lastDirDelimiter+1:end );
end

% Look for digits
listDigits = regexp( leftOver, '[0-9]' );

if isempty( listDigits )
  imageInfo.Prefix = leftOver;
  imageInfo.nbVal  = 0;
  imageInfo.Start  = 0;
  imageInfo.End    = 0;
else
  lastDigitIndex = listDigits( end );
  if( lastDigitIndex ~= length( leftOver ) )
    % Add remaining vals to suffix
    imageInfo.Suffix = [ leftOver( lastDigitIndex+1:end  ) imageInfo.Suffix];
    leftOver = leftOver( 1:lastDigitIndex );
  end
    
  % Look for continuous block of digits
  firstDigitIndex = lastDigitIndex;
  listDigits = listDigits( 1:end-1 );

  while( ~isempty( listDigits ) && listDigits( end ) == firstDigitIndex - 1 )
    firstDigitIndex = firstDigitIndex - 1;
    listDigits = listDigits( 1:end-1 );
  end
  
  imageInfo.Prefix = leftOver( 1:firstDigitIndex-1 );
  imageInfo.nbVal  = lastDigitIndex - firstDigitIndex + 1;
  imageInfo.Start  = str2num( leftOver( firstDigitIndex:lastDigitIndex ) );

  if( bFindEnd ) 
    searchImages = [imageInfo.Path imageInfo.Prefix '*' imageInfo.Suffix ];
    listFiles = dir( searchImages );
    if( isempty( listFiles ) )
      fprintf( 'ERROR in "discoverImageNameFormat" looking for last image: could not find any images in directory.' );
      return
    end
    lastFile = listFiles( end ).name;
    % Remove suffix and prefix
    begSuffixIndex = strfind( lastFile, imageInfo.Suffix );
    lastFile = lastFile( 1:begSuffixIndex-1 );
    
    digitLastFile = [];
    while( ~isempty( lastFile ) && ~isempty( str2num( lastFile( end ) )) )
      digitLastFile = [ lastFile( end ) digitLastFile ];
      lastFile = lastFile( 1:end-1 );
    end
    imageInfo.End = str2num( digitLastFile );
  end
end