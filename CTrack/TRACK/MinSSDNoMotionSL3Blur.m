% function estimates = MinSSDNoMotionSL3Blur( IRef, ICur, template, estimates, minInfo, bDisp )
%
% Find the blur to apply to a patch in IRef to obtain a patch in
% ICur:
% min_{MEst} \sum |ICur( HEst x ) - \int IRef( HRef expm( MEst t ) x ) dt |
%
% To simplify parameter passing, 3 structures as used as described
% in the following.
%
% Input:
% - IRef: reference unblurred image from which the template to track is extracted
% - ICur: current blurred image in which we wish to track the template
% - template: structure containing information on the template to track:
%   * template.nWidth: template width
%   * template.nHeight: template height
% - estimates:
%   * estimates.HEst: estimated homography to match the template
%   between IRef and ICur.
%   * estimates.MEst: estimated blur in Lie algebra space
% - minInfo: structure containing parameters pertaining to the minimisation:
%   * minInfo.nMaxIter: max. number of iterations
%   * minInfo.nNumParametersM: number of parameters to estimate for
%   the blur (2: translation, 6: affine, 8: homography)
%   * minInfo.bRobust: should m-estimators be used (Huber)
% - debug (optional): 
%   * debug.bDisp: display initial and current template
%   * debug.bVerbose: print RMS
%
% Output:
% - estimates: same structure as input but MEst has been updated.
%
function estimates = MinSSDNoMotionSL3Blur( IRef, ICur, template, estimates, minInfo, debug )

if nargin == 0
  fprintf( 'Launching test...\n');
  test();
  return;
end
  
if nargin < 6
  debug.bDisp = 0;
  debug.bVerbose = 0;
end

HRef            = template.HRef;
nWidth          = template.nWidth;
nHeight         = template.nHeight;

HEst            = estimates.HEst;
MEst            = estimates.MEst;

nMaxIter        = minInfo.nMaxIter;
nNumParametersM = minInfo.nNumParametersM;
bRobust         = minInfo.bRobust;

bDisp           = debug.bDisp;
bVerbose        = debug.bVerbose;

[TICur,TICurMask] = warping( ICur, HRef*estimates.HEst, nWidth, nHeight );

if bDisp
  fig = figure;
  TIRefEst = blur_warping( IRef, HRef, MEst, eye(3), nWidth, nHeight );
  subplot( 2, 2, 1 )
  imshow( uint8( TIRefEst ) )
  title( 'Reference template blurred with initial MEst' )

  subplot( 2, 2, 2 )
  imshow( uint8( TICur ) )
  title( 'Current template extracted from blurred image' )
end

bestRMS  = Inf;
MEstPrev = MEst;
MEstCur  = MEst;
MEstBest = MEst;

dMinNormUpdate = 1e-6;

for nIter=1:nMaxIter
  [MEstCur,RMS,dNormUpdate,TIRefEst] = ...
      SSDUpdateNoMotionSL3Blur( IRef, TICur, TICurMask, HRef, MEstPrev, nNumParametersM, bRobust );

  % The RMS correponds to the HEstPrev estimate
  if RMS < bestRMS
    bestRMS  = RMS;
    MEstBest = MEstPrev;
    MEstBest = MEstPrev;
  end
   
  MEstPrev = MEstCur;
  
  if dNormUpdate < dMinNormUpdate
    break;
  end
  
  if bVerbose
    fprintf( ' Iter %i, RMS: %f, bestRMS: %f\n', nIter, RMS, bestRMS );
  end
  
  if bDisp
    figure(fig)
    subplot( 2, 2, 3 )
    imshow( uint8( TIRefEst ) )
    title( 'Blurred reference template' )
    subplot( 2, 2, 4 )
    imshow( uint8( TICur ) )
    cur_title = sprintf( 'Current template (RMS: %3.2f)', RMS ); 
    title( cur_title )
    drawnow
    %pause
  end
end  

estimates.MEst = MEstBest;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( M, nNumParametersM )

if nargin == 0  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make blur matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %blur_type = 'rot';
  blur_type = 'zoom';
  %blur_type = 'trans';

  switch blur_type
    case 'rot'
      nNumParametersM = 5;
      
      rot_ax = [0;0;1];
      angle = 10*pi/180;
      M = angle*skew(rot_ax);
      N = M*[100;100;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
    case 'zoom'
      nNumParametersM = 8;
      
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.2;
      M(2,2) = M(2,2)-0.2;
      N = M*[150/2;150/2;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
    case 'trans'
      nNumParametersM = 2;

      M = zeros(3,3);
      M(1,3) = M(1,3)+10;
      M(2,3) = M(2,3);
  end 

  test( M, nNumParametersM );
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

HRef = [ 1 0 cx;
         0 1 cy;
         0 0 1 ];

%
% Build simulation values: see last section of TRO article
%
fprintf( 'Building simulation images, this can take some time... ' );
IRef = I;
ICur = blur_warping( I, HRef, M, inv(HRef), nImageWidth, nImageHeight );

TIRef = blur_warping( IRef, HRef, M, eye(3), nWidth, nHeight );
TICur = warping( ICur, HRef, nWidth, nHeight );

absDiff = abs( TIRef(:) - TICur(:) );

fprintf( 'done.\n' );

fprintf( 'Simulation errors: max: %f, RMS: %f, median: %f.\n', ...
         max( absDiff ), sqrt( mean( absDiff.^2  ) ), median( absDiff ) );

% Setup parameters
template.HRef    = HRef;
template.nWidth  = nWidth;
template.nHeight = nHeight;

estimates.HEst   = eye(3);
estimates.MEst   = zeros(3);

minInfo.nMaxIter        = 15;
minInfo.nNumParametersM = nNumParametersM;
minInfo.bRobust         = 0;

debug.bDisp    = 1;
debug.bVerbose = 1;

estimates = MinSSDNoMotionSL3Blur( IRef, ICur, template, estimates, minInfo, debug );

M
estimates.MEst
