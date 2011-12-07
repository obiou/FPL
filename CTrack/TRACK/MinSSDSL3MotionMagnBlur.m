% function estimates = MinSSDSL3MotionMagnBlur( IRef, ICur, template, estimates, minInfo, debug )
%
% Tracks a template between two images IRef and ICur by
% sum-of-square differences.
%
% To simplify parameter passing, 4 structures as used as described
% in the following.
%
% Input:
% - IRef: reference image from which the template to track is extracted
% - ICur: current image in which we wish to track the template
% - ICurMask: current image mask
% - template: structure containing information on the template to track:
%   * template.HRef: homography for the reference template
%   extractio (typically translation to the top-left corner of the
%   template in IRef)
%   * template.nWidth: template width
%   * template.nHeight: template height
% - estimates:
%   * estimates.HPrev: previous estimated homography (to ensure the blur only uses the update inv(HPrev)*HEst)
%   * estimates.HEst: estimated homography
%   * estimates.lambdaEst: current blur magnitude estimate
% - minInfo: structure containing parameters pertaining to the minimisation:
%   * minInfo.nMaxIter: max. number of iterations
%   * minInfo.nNumParametersH: number of parameters to estimate for
%     H (2: translation, 6: affine, 8: homography)
%   * minInfo.bRobust: should m-estimators be used (Huber)?
% - debug (optional): 
%   * debug.bDisp: display initial and current template
%   * debug.bVerbose: print RMS
%
% Output:
% - estimates: same structure as input but HEst and lambdaEst have been updated.
%
function estimates = MinSSDSL3MotionMagnBlur( IRef, ICur, ICurMask, template, estimates, minInfo, debug )

if nargin == 0
  fprintf( 'Launching test...\n');
  test();
  return;
end
  
if nargin < 7
  debug.bDisp = 0;
  debug.bVerbose = 0;
end

HRef            = template.HRef;
nWidth          = template.nWidth;
nHeight         = template.nHeight;

HPrev           = estimates.HPrev;
HEst            = estimates.HEst;
lambdaEst       = estimates.lambdaEst;

nMaxIter        = minInfo.nMaxIter;
nNumParametersH = minInfo.nNumParametersH;
bHPrev          = minInfo.bHPrev;
bRobust         = minInfo.bRobust;

bDisp           = debug.bDisp;
bVerbose        = debug.bVerbose;

if bDisp
  if bHPrev
    TIRef = blur_warping( IRef, HRef*HPrev, lambdaEst*logm( inv(HPrev)*HEst ), HPrev, nWidth, nHeight );
  else
    TIRef = blur_warping( IRef, HRef, lambdaEst*logm( inv(HPrev)*HEst ), eye(3), nWidth, nHeight );
  end
  TICur = warping( ICur, HRef*HEst, nWidth, nHeight );
  
  fig = figure;
  subplot( 2, 2, 1 )
  imshow( uint8( TIRef ) )
  title( 'Blurred reference template using initial value of lambdEst' )

  subplot( 2, 2, 2 )
  imshow( uint8( TICur ) )
  title( 'Current template using initial value of HEst' )
end

bestRMS       = Inf;
HEstPrev      = HEst;
HEstCur       = HEst;
HEstBest      = HEst;
lambdaEstPrev = lambdaEst;
lambdaEstCur  = lambdaEst;
lambdaEstBest = lambdaEst;

dMinNormUpdate = 1e-6;

for nIter=1:nMaxIter
  [HEstCur,lambdaEstCur,RMS,dNormUpdate,TIRefEst,TICurEst] = ...
      SSDUpdateSL3MotionMagnBlur( IRef, ICur, ICurMask, HRef, HPrev, nWidth, nHeight, HEstPrev, lambdaEstPrev, ...
                                  nNumParametersH, bHPrev, bRobust );

  % The RMS correponds to the HEstPrev estimate
  if RMS < bestRMS
    bestRMS       = RMS;
    HEstBest      = HEstPrev;
    lambdaEstBest = lambdaEstPrev;
    TIRefBest     = TIRefEst;
    TICurBest     = TICurEst;
  end
   
  HEstPrev      = HEstCur;
  lambdaEstPrev = lambdaEstCur;
  
  if dNormUpdate < dMinNormUpdate
    break;
  end
  
  if bVerbose
    fprintf( ' Iter %i, RMS: %f, bestRMS: %f\n', nIter, RMS, bestRMS );
  end
  
  if bDisp
    figure( fig )
    subplot( 2, 2, 3 )
    imshow( uint8( TIRefEst ) )
    title( 'Blurred reference template' )
    subplot( 2, 2, 4 )
    imshow( uint8( TICurEst ) )
    cur_title = sprintf( 'Current template (RMS: %3.2f)', RMS ); 
    title( cur_title )
    drawnow
    %pause
  end
end  

estimates.HEst      = HEstBest;
estimates.lambdaEst = lambdaEstBest;
estimates.RMS       = bestRMS;
estimates.TIRef     = TIRefBest;
%estimates.TIRefMask = TIRefMaskBest;
estimates.TICur     = TICurBest;
%estimates.TICurMask = TICurMaskBest;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( Hi, lambda, nNumParametersH )

if nargin == 0  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make homography matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %sl3_type = 'rot';
  sl3_type = 'zoom';
  %sl3_type = 'trans';

  switch sl3_type
    case 'rot'
      nNumParametersH = 5;
      
      rot_ax = [0;0;1];
      angle = 10*pi/180;
      M = angle*skew(rot_ax);
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      Hi = expm( M );
    case 'zoom'
      nNumParametersH = 8;
      
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.2;
      M(2,2) = M(2,2)-0.2;
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      Hi = expm( M );
      Hi = Hi/det(Hi)^(1/3);
    case 'trans'
      nNumParametersH = 2;

      Hi = eye(3);
      Hi(1,3) = Hi(1,3)+20;
      Hi(2,3) = Hi(2,3);
  end 
  lambda = 0.5;

  test( Hi, lambda, nNumParametersH );
  return
end

load mandrill;
I            = X;
nImageWidth  = size( I, 2 );
nImageHeight = size( I, 1 );
nWidth       = 150;
nHeight      = 150;
cx           = nImageWidth/2 - nWidth/2;
cy           = nImageHeight/2 - nHeight/2 + 70;
bHPrev       = 1;

HRef = [ 1 0 cx;
         0 1 cy;
         0 0 1 ];

HPrev = [ 1 0 20;
          0 1 -15;
          0 0 1];

H = Hi*HPrev; % WARNING: prev should be on the right (H(x) is only
              % put on the right for practical minimisation reasons)

%
% Build simulation values: see last section of TRO article
%
% Add blur to avoid aliasing effects due to double warping
% to build the current image and then current template
h = fspecial( 'gaussian', 5, 5 );
%I = imfilter( I, h );

fprintf( 'Building simulation images, this can take some time... ' );

IRef  = I;
if bHPrev
  [ ICur, ICurMask ] = blur_warping( I, HRef*inv(HPrev), lambda*logm( Hi ), inv( HRef * Hi), nImageWidth, nImageHeight );
  TIRef              = blur_warping( IRef, HRef*inv(HPrev), lambda*logm( Hi ), HPrev, nWidth, nHeight );
else
  [ ICur, ICurMask ] = blur_warping( I, HRef, lambda*logm( Hi ), inv( HRef*H ), nImageWidth, nImageHeight );
  TIRef              = blur_warping( IRef, HRef, lambda*logm( Hi ), eye(3), nWidth, nHeight );
end

TICur = warping( ICur, HRef*H, nWidth, nHeight );

fprintf( 'done.\n' );

absDiff = abs( TIRef(:) - TICur(:) );

fprintf( 'Simulation errors: max: %f, RMS: %f, median: %f.\n', ...
         max( absDiff ), sqrt( mean( absDiff.^2  ) ), median( absDiff ) );

% Setup parameters
template.HRef           = HRef;
template.nWidth         = nWidth;
template.nHeight        = nHeight;

estimates.HPrev         = HPrev;
estimates.HEst          = HPrev;
estimates.lambdaEst     = 0.5;

minInfo.nMaxIter        = 15;
minInfo.nNumParametersH = nNumParametersH;
minInfo.bHPrev          = bHPrev;
minInfo.bRobust         = 0;

debug.bDisp             = 1;
debug.bVerbose          = 1;

estimates = MinSSDSL3MotionMagnBlur( IRef, ICur, ICurMask, template, estimates, minInfo, debug );

H
estimatedH = estimates.HEst
lambda
estimatedLambda = estimates.lambdaEst
