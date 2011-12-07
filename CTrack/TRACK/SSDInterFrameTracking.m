%
% Main function for tracking between frames (no templates).
%
% Input structures:
% - IRef: initial image/image name
% - ICur: current image/image name
% - ICurMask: current image mask
% - minInfo: (optional) structure containing the type of tracking required and the parameters
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
% - saveInfo: (optional)
%   * saveInfo.bSave: boolean indicating if we wish to save the tracked templates and estimates
%   * saveInfo.sSaveDir: directory to save the data
% - debug: (optional)
%   debug.bDisp    : display results
%   debug.bVerbose : show RMS
% - estimates: (optional) see output for details
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
function estimates = SSDInterFrameTracking( IRef, ICur, ICurMask, minInfo, saveInfo, debug, estimates )
  
if nargin == 0
  fprintf( 'Expecting at least two image names as input.\n');
  return
end

if isstr( IRef )
  IRef = double( imread( IRef ) );
end

if isstr( ICur )
  ICur = double( imread( ICur ) );
  ICurMask = ones( size( ICur ) );
end

if ~isfloat( IRef )
  IRef = double( IRef );
end

if ~isfloat( ICur )
  ICur = double( ICur );
end

if nargin < 4
  minInfo = defaultTrackingParameters();
end

if nargin < 5
  saveInfo.bSave = 0;
end

if nargin < 6
  debug.bDisp = 0;
  debug.bVerbose = 1;
end

if nargin < 7
  estimates = InitialiseEstimates();
end

%IRefInit = IRef;
%ICurInit = ICur;

IRefInit = imresize( IRef, minInfo.dResizeImageFactor );
ICurInit = imresize( ICur, minInfo.dResizeImageFactor );
ICurInitMask = imresize( ICurMask, minInfo.dResizeImageFactor );

template.HRef = [ 1 0 0; 
                  0 1 0; 
                  0 0 1 ];
nWidth  = size( IRefInit, 2 );
nHeight = size( IRefInit, 1 );
template.nWidth  = nWidth;
template.nHeight = nHeight;

fig_main = figure();
subplot( 2, 3, 1 )
imshow( uint8( IRef ) );
title( 'Reference image' )

subplot( 2, 3, 2 )
imshow( uint8( ICur ) );
title( 'Current image' )

drawnow

size( IRefInit )

SelTracker = SelectTracker( minInfo.sTrackingType, minInfo.bIllum );
estimates  = BuildEstimates( estimates );

for scaleIndex=1:length(minInfo.Scales)
  if minInfo.Scales(scaleIndex) ~= 0
    ker = fspecial( 'gaussian', minInfo.Scales(scaleIndex), minInfo.Scales(scaleIndex) );
    IRefPyr( :, :, scaleIndex ) = imfilter( IRefInit, ker );
  end
end

for scaleIndex=1:length(minInfo.Scales)
  if minInfo.Scales( scaleIndex ) == 0
    IRef = IRefInit;
    ICur = ICurInit;
  else
    IRef = IRefPyr( :, :, scaleIndex );
    ker = fspecial( 'gaussian', minInfo.Scales(scaleIndex), minInfo.Scales(scaleIndex) );
    ICurInitThresh = ICurInit;
    ICurInitThresh( find( ICurInitMask == 0 ) ) = NaN;
    ICur = imfilter( ICurInitThresh, ker );
  end
  
  %estimates = SelTracker( IRef, ICur, template, estimates, minInfo, debug );
  estimates = SelTracker( IRef, ICur, ICurInitMask, template, estimates, minInfo, debug );
  
  figure( fig_main );
  subplot( 2, 3, 4 )
  imshow( uint8( estimates.TIRef ) );
  title( 'Reference' )
  drawnow;

  subplot( 2, 3, 5 )
  imshow( uint8( estimates.TICur ) );
  title( 'Warped current' )
  title( sprintf( 'Warped current, RMS: %3.2f ', estimates.RMS ) );
  drawnow;

  subplot( 2, 3, 6 )
  IDiff = estimates.TICur - estimates.TIRef;
  %IDiff( find( estimates.TICurMask == 0 | estimates.TIRefMask == 0 ) ) = 0;
  imagesc( abs( IDiff ) ); colorbar; colormap( gray );
  axis image;
  title( 'Abs difference' )
  drawnow;
  
  if 0
    figure( fig_cur )
    subplot( 1, 2, 1 )
    imshow( uint8( warping( ICur, template.HRef*estimates.HEst, template.nWidth, template.nHeight ) ) )
    subplot( 1, 2, 2 )
    imshow( uint8( estimates.TICur ) );
  end
  drawnow;
  %pause
end

if saveInfo.bSave
  estimates.lambdaEst
  imwrite( uint8( estimates.TIRef ), [ saveInfo.sSaveDir sprintf( '/ref_%03d.png', imageNumber ) ] );
  imwrite( uint8( estimates.TICur ), [ saveInfo.sSaveDir sprintf( '/cur_%03d.png', imageNumber ) ] );
  save( [ saveInfo.sSaveDir '/estimates.mat' ], 'estimates' );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function minInfo = defaultTrackingParameters()

minInfo.nMaxIter          = 50;
minInfo.nNumParametersH   = 8;
minInfo.nNumParametersM   = 8;
minInfo.bPreMinH          = 0;
minInfo.bPreMinHMaxIter   = 5;
minInfo.bPreMinM          = 0;
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
estimates.lambdaEst = 0.5;
estimates.alphaEst  = 1;
estimates.betaEst   = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function estimates = BuildEstimates( estimates )
if  ~isfield( estimates, 'RMS' )
  estimates.RMS = [];
end 
if ~isfield( estimates, 'HPrev' )
  estimates.HPrev     = eye(3);
end
if ~isfield( estimates, 'HEst' )
  estimates.HEst      = eye(3);
end
if ~isfield( estimates, 'MEst' )
  estimates.MEst      = zeros(3);
end
if ~isfield( estimates, 'lambdaEst' )
  estimates.lambdaEst = 1;
end
if ~isfield( estimates, 'alphaEst' )
  estimates.alphaEst  = 1;
end
if ~isfield( estimates, 'betaEst' )
  estimates.betaEst   = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function SelTracker = SelectTracker( sTrackingType, bIllum )
switch sTrackingType
  case 'SL3Motion'
    if bIllum
      SelTracker = @( IRef, ICur, ICurMask, template, estimates, minInfo, debug ) MinSSDSL3MotionAffineIllumination( IRef, ICur, ICurMask, template, estimates, minInfo, debug );
    else
      SelTracker = @( IRef, ICur, template, estimates, minInfo, debug ) MinSSDSL3Motion( IRef, ICur, template, estimates, minInfo, debug );
    end
  case 'SL3MotionSL3Blur'
    if bIllum
      SelTracker = @( IRef, ICur, template, estimates, minInfo, debug ) MinSSDSL3MotionSL3BlurAffineIllumination( IRef, ICur, template, estimates, minInfo, debug );
    else
      SelTracker = @( IRef, ICur, template, estimates, minInfo, debug ) MinSSDSL3MotionSL3Blur( IRef, ICur, template, estimates, minInfo, debug );
    end
  case 'SL3MotionBlurMagn'
    if bIllum
      SelTracker = @( IRef, ICur, ICurMask, template, estimates, minInfo, debug ) MinSSDSL3MotionMagnBlurAffineIllumination( IRef, ICur, ICurMask, template, estimates, minInfo, debug );
    else
      SelTracker = @( IRef, ICur, template, estimates, minInfo, debug ) MinSSDSL3MotionMagnBlur( IRef, ICur, template, estimates, minInfo, debug );
    end
  otherwise
    error( 'TYPE', fprintf( '%s tracking type not yet implemented.\n', minInfo.sTrackingType ) )
end