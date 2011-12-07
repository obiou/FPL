%
% Main function for tracking a sequence.
%
% Input structures:
% - imageInfo: image name or structure created by "discoverImageNameFormat"
% - template: one of the following:
%   * winPos: four corners describing the rectangle to track
%   * HRef, nWidth, nHeight: template in terms of warp TIRef = warping( IRef, HRef, nWidth, nHeight )
%   * empty ([]): this will trigger a call for an input
% - minInfo: structure containing the type of tracking required and the parameters
%            default values are given if this is empty (tracking is
%            then homography only)
%   * minInfo.nMaxIter       :
%   * minInfo.nNumParametersH: 
%   * minInfo.nNumParametersM: 
%   * minInfo.bPreMinH       : 
%   * minInfo.bPreMinHMaxIter: 
%   * minInfo.bPreMinM       : 
%   * minInfo.bPreMinMMaxIter: 
%   * minInfo.bRobust        : 
%   * minInfo.sTrackingType  : 'SL3Motion'/'SL3MotionSL3Blur'/'SL3MotionBlurMagn'
% - saveInfo:
%   * saveInfo.bSave: boolean indicating if we wish to save the tracked templates and estimates
%   * saveInfo.sSaveDir: directory to save the data
% - debug:
%   debug.bDisp    : display results
%   debug.bVerbose : show RMS
%
% Output: (some of these values will not be updated if the called type does not estimate it!)
% - estimates:
%   * estimates.RMS: list of RMS for each tracked image
%   * estimates.HEst: estimated homography
%   * estimates.MEst: estimated blur Lie algebra value
%   * estimates.lambdaEst: estimated blur magnitude
%   * estimates.alpha: estimated intensity multiplicative factor (affine illumination model)
%   * estimates.beta: estimated intensity offset (affine illumination model)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function estimates = SSDTracking( imageInfo, template, minInfo, saveInfo, debug )
  
if nargin == 0
  fprintf( 'Expecting at least an image name as input.\n');
  return
end

% Hack: if the input does not contain 'Start' assume it is just an
% image name.
if ~isfield( imageInfo, 'Start' )
  imageInfo = discoverImageNameFormat( imageInfo );
  estimates = SSDTracking( imageInfo );
  return
end

if nargin < 2
  template = [];
end

if nargin < 3
  minInfo = defaultTrackingParameters();
end

if nargin < 4
  saveInfo.bSave = 0;
end

if nargin < 5
  debug.bDisp = 0;
  debug.bVerbose = 1;
end

IRef = imageLoad( imageInfo, imageInfo.Start );

bTemplateHRefForm   = isfield( template, 'HRef' ) && isfield( template, 'nWidth' ) && isfield( template, 'nHeight' );
bTemplateWinPosForm = isfield( template, 'winPos' );

fig_main = figure;

if ~bTemplateHRefForm && ~bTemplateWinPosForm
  figure( fig_main )
  subplot( 1, 2, 1 )
  imshow( uint8( IRef ) );
  title( 'Click on 2 points corresponding to the corners of the rectangle to track.');
  [clX,clY] = ginput(2);
  template.winPos = [floor( min(clX) ) floor( min(clY) )  ceil( abs(clX(1)-clX(2)) ) ceil( abs(clY(1)-clY(2)) )];
end

if ~bTemplateHRefForm
  winPos = template.winPos;
  template.HRef = [ 1 0 winPos( 1 ); 
                    0 1 winPos( 2 ); 
                    0 0 1 ];
  template.nWidth  = winPos( 3 );
  template.nHeight = winPos( 4 );
  template.Corners = [ 1 winPos( 3 ) winPos( 3 ) 1 1;
                      1 1 winPos( 4 ) winPos( 4 ) 1; 
                      ones( 1, 5 ) ];
end

estimates = InitialiseEstimates();

subplot( 2, 3, 1 )
imshow( uint8( IRef ) ); hold on;
Corners = metric( template.HRef*template.Corners );
plot( Corners(1,:), Corners(2,:), 'r-' );

subplot( 2, 3, 3 )
imshow( uint8( warping( IRef, template.HRef, template.nWidth, template.nHeight ) ) );
title( 'Selected template' )

if 0
  fig_cur = figure;
end

nNumImages = imageInfo.End - imageInfo.Start +1;
IRefInit = IRef;

for scaleIndex=1:length(minInfo.Scales)
  if minInfo.Scales(scaleIndex) ~= 0
    ker = fspecial( 'gaussian', minInfo.Scales(scaleIndex), minInfo.Scales(scaleIndex) );
    IRefPyr( :, :, scaleIndex ) = imfilter( IRefInit, ker );
  end
end

% Start the tracking
for imageNumber=imageInfo.Start:imageInfo.End
  ICurInit = imageLoad( imageInfo, imageNumber );
  
  imageIndex = imageNumber - imageInfo.Start + 1;
  localEstimates = BuildLocalEstimates( estimates, imageIndex, minInfo.bConstantVelocity );
  
  for scaleIndex=1:length(minInfo.Scales)
    if minInfo.Scales(scaleIndex) == 0
      IRef = IRefInit;
      ICur = ICurInit;
    else
      IRef = IRefPyr( :, :, scaleIndex );
      ker = fspecial( 'gaussian', minInfo.Scales(scaleIndex), minInfo.Scales(scaleIndex) );
      ICur = imfilter( ICurInit, ker );
    end
    switch minInfo.sTrackingType
      case 'SL3Motion'
        if minInfo.bIllum
          localEstimates = MinSSDSL3MotionAffineIllumination( IRef, ICur, template, localEstimates, minInfo, debug );
        else
          localEstimates = MinSSDSL3Motion( IRef, ICur, template, localEstimates, minInfo, debug );
        end
      case 'SL3MotionSL3Blur'
        if minInfo.bIllum
          localEstimates = MinSSDSL3MotionSL3BlurAffineIllumination( IRef, ICur, template, localEstimates, minInfo, debug );
        else
          localEstimates = MinSSDSL3MotionSL3Blur( IRef, ICur, template, localEstimates, minInfo, debug );
        end
      case 'SL3MotionBlurMagn'
        if minInfo.bIllum
          localEstimates = MinSSDSL3MotionMagnBlurAffineIllumination( IRef, ICur, template, localEstimates, minInfo, debug );
        else
          localEstimates = MinSSDSL3MotionMagnBlur( IRef, ICur, template, localEstimates, minInfo, debug );
        end
      otherwise
        error( 'TYPE', fprintf( '%s tracking type not yet implemented.\n', minInfo.sTrackingType ) )
    end

    figure( fig_main );
    subplot( 2, 3, 2 );
    imshow( uint8( ICur ) ); hold on;
    estimates.Corners(:,:,imageIndex) = metric( template.HRef*localEstimates.HEst*template.Corners );
    plot( estimates.Corners(1,:,imageIndex), estimates.Corners(2,:,imageIndex), 'r-' );
    title( sprintf( '%i/%i, RMS: %3.2f ', imageIndex, nNumImages, localEstimates.RMS ) );

    subplot( 2, 3, 4 )
    imshow( uint8( localEstimates.TICur ) );
    title( 'Warped current')
    
    subplot( 2, 3, 5 )
    imshow( uint8( localEstimates.TIRef ) );
    title( 'Reference')

    subplot( 2, 3, 6 )
    imagesc( abs( localEstimates.TICur - localEstimates.TIRef ) ); colorbar; colormap( gray );
    axis image;
    title( 'Abs difference')
    
    if 0
      figure( fig_cur )
      subplot( 1, 2, 1 )
      imshow( uint8( warping( ICur, template.HRef*localEstimates.HEst, template.nWidth, template.nHeight ) ) )
      subplot( 1, 2, 2 )
      imshow( uint8( localEstimates.TICur ) );
    end
    drawnow;
    %pause
    
    estimates = UpdateEstimates( estimates, localEstimates, imageIndex );
  end
  
  if saveInfo.bSave
    estimates.lambdaEst
    imwrite( uint8( localEstimates.TIRef ), [ saveInfo.sSaveDir sprintf( '/ref_%03d.png', imageNumber ) ] );
    imwrite( uint8( localEstimates.TICur ), [ saveInfo.sSaveDir sprintf( '/cur_%03d.png', imageNumber ) ] );
    save( [ saveInfo.sSaveDir '/estimates.mat' ], 'estimates' );
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function minInfo = defaultTrackingParameters()

minInfo.nMaxIter          = 50;
minInfo.nNumParametersH   = 8;
minInfo.nNumParametersM   = 8;
minInfo.bPreMinH          = 1;
minInfo.bPreMinHMaxIter   = 5;
minInfo.bPreMinM          = 1;
minInfo.bPreMinMMaxIter   = 5;
minInfo.bRobust           = 0;
minInfo.Scales            = [40;20;10;5;1];
minInfo.bConstantVelocity = 1;
minInfo.bIllum            = 1;
minInfo.sTrackingType     = 'SL3Motion';
%minInfo.sTrackingType     = 'SL3MotionSL3Blur';
%minInfo.sTrackingType     = 'SL3MotionBlurMagn';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function estimates = InitialiseEstimates()

estimates.RMS       = [];
estimates.HEst      = eye(3);
estimates.MEst      = zeros(3);
estimates.lambdaEst = 1;
estimates.alphaEst  = 1;
estimates.betaEst   = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 'index' corresponds to the current image (that does not have any
% estimated estimates yet)
%
function localEstimates = BuildLocalEstimates( estimates, index, bConstantVelocity  )

if index > 1
  localEstimates.HPrev   = estimates.HEst( :, :, index - 1 );
else
  localEstimates.HPrev   = eye(3);
end

if bConstantVelocity && index > 2
  lieHEstPrev = HToLie( estimates.HEst( :, :, index - 2 ) );
  lieHEstCur  = HToLie( estimates.HEst( :, :, index - 1 ) );
  
  localEstimates.HEst    = LieToH( lieHEstCur + ( lieHEstCur - lieHEstPrev ) );
else
  localEstimates.HEst  = estimates.HEst( :, :, index );
end

localEstimates.MEst      = estimates.MEst( :, :, index );
localEstimates.lambdaEst = estimates.lambdaEst( index );
localEstimates.alphaEst  = estimates.alphaEst( index );
localEstimates.betaEst   = estimates.betaEst( index );

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function estimates = UpdateEstimates( estimates, localEstimates, index )

estimates.RMS( index )            = localEstimates.RMS;

estimates.HEst( :, :, index )     = localEstimates.HEst;
estimates.HEst( :, :, index + 1 ) = localEstimates.HEst;

estimates.MEst( :, :, index )     = localEstimates.MEst;
estimates.MEst( :, :, index + 1 ) = localEstimates.MEst;

estimates.lambdaEst( index )      = localEstimates.lambdaEst;
estimates.lambdaEst( index + 1 )  = localEstimates.lambdaEst;

estimates.alphaEst( index )       = localEstimates.alphaEst;
estimates.alphaEst( index + 1 )   = localEstimates.alphaEst;

estimates.betaEst( index )        = localEstimates.betaEst;
estimates.betaEst( index + 1 )    = localEstimates.betaEst;
