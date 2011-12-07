% function estimates = MinSSDSL3Motion( IRef, ICur, template, estimates, minInfo, bDisp )
%
% Tracks a template between two images IRef and ICur by
% sum-of-square differences.
%
% To simplify parameter passing, 3 structures are used as described
% in the following.
%
% Input:
% - IRef: reference image from which the template to track is extracted
% - ICur: current image in which we wish to track the template
% - template: structure containing information on the template to track:
%   * template.HRef: homography for the reference template
%   extraction (typically translation to the top-left corner of the
%   template in IRef)
%   * template.nWidth: template width
%   * template.nHeight: template height
% - estimates: structure containing the estimates
%   * estimates.HEst: contains the estimated homography
%   * estimates.alphaEst: multiplicative factor in affine illumination model
%   * estimates.betaEst: additive factor in affine illumination model
% - minInfo: structure containing parameters pertaining to the minimisation:
%   * minInfo.nMaxIter: max. number of iterations
%   * minInfo.nNumParametersH: number of parameters to estimate (2: translation, 6: affine, 8: homography)
%   * minInfo.bRobust: should m-estimators be used (Huber)
% - debug (optional): 
%   * debug.bDisp: display initial and current template
%   * debug.bVerbose: print RMS
%
% Output:
% - estimates: same structure as input but HEst has been updated.
%
function estimates = MinSSDSL3MotionAffineIllumination( IRef, ICur, ICurMask, template, estimates, minInfo, debug )

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
alphaEst        = estimates.alphaEst;
betaEst         = estimates.betaEst;

nMaxIter        = minInfo.nMaxIter;
nNumParametersH = minInfo.nNumParametersH;
bRobust         = minInfo.bRobust;

bDisp           = debug.bDisp;
bVerbose        = debug.bVerbose;

TIRef = warping( IRef, HRef, nWidth, nHeight );

if bDisp
  fig = figure;
  subplot( 1, 2, 1 )
  imshow( uint8( TIRef ) )
  title( 'Reference template' )
end

bestRMS      = Inf;
HEstPrev     = HEst;
HEstCur      = HEst;
HEstBest     = HEst;
alphaEstPrev = alphaEst;
alphaEstCur  = alphaEst;
alphaEstBest = alphaEst;
betaEstPrev  = betaEst;
betaEstCur   = betaEst;
betaEstBest  = betaEst;

dMinNormUpdate = 1e-8;

for nIter=1:nMaxIter
  [HEstCur,alphaEstCur,betaEstCur,RMS,dNormUpdate,TICur] = ...
      SSDUpdateSL3MotionAffineIllumination( TIRef, ICur, ICurMask, HRef, ...
                                            HEstPrev, nNumParametersH, ...
                                            alphaEstPrev, betaEstPrev, bRobust );

  % The RMS correponds to the HEstPrev estimate
  if RMS < bestRMS
    bestRMS      = RMS;
    HEstBest     = HEstPrev;
    alphaEstBest = alphaEstPrev;
    betaEstBest  = betaEstPrev;
    TICurBest    = TICur;
  end
   
  HEstPrev     = HEstCur;
  alphaEstPrev = alphaEstCur;
  betaEstPrev  = betaEstCur;
  
  if dNormUpdate < dMinNormUpdate
    break;
  end
  if bVerbose
    fprintf( ' Iter %i, RMS: %f, bestRMS: %f\n', nIter, RMS, bestRMS);
  end
  if bDisp
    figure(fig)
    subplot( 1, 2, 2 )
    imshow( uint8( TICur ) )
    cur_title = sprintf( 'Current template (RMS: %3.2f)', RMS ); 
    title( cur_title )
    drawnow
    %pause
  end
end  

estimates.HEst     = HEstBest;
estimates.alphaEst = alphaEstBest;
estimates.betaEst  = betaEstBest;
estimates.RMS      = bestRMS;
estimates.TICur    = TICurBest;
estimates.TIRef    = TIRef;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( H, nNumParametersH, alpha, beta )

if nargin == 0
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %sl3_type = 'rot';
  %sl3_type = 'zoom';
  sl3_type = 'trans';

  switch sl3_type
    case 'rot'
      nNumParametersH = 5;
      
      rot_ax = [0;0;1];
      angle = 5*pi/180;
      M = angle*skew(rot_ax);
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
    case 'zoom'
      nNumParametersH = 8;
      
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.05;
      M(2,2) = M(2,2)-0.1;
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
    case 'trans'
      nNumParametersH = 2;

      H = eye(3);
      H(1,3) = H(1,3)-15;
      H(2,3) = H(2,3)+5;
  end 
  alpha = 1.2;
  beta  = 30;

  test( H, nNumParametersH, alpha, beta );
  return
end

load mandrill;
I = X;

nWidth  = 150;
nHeight = 200;
cx      = size( I, 1 )/2 - nWidth/2;
cy      = size( I, 2 )/2 - nHeight/2;

HRef = [ 1 0 cx;
         0 1 cy;
         0 0 1 ];

IRef = alpha * warping( I, HRef*H*inv(HRef), size(I,2), size(I,1) ) + beta;
ICur = I;
ICurMask = ones( size( I ) );

% Setup parameters
template.HRef           = HRef;
template.nWidth         = nWidth;
template.nHeight        = nHeight;

estimates.HEst          = eye(3);
estimates.alphaEst      = 1;
estimates.betaEst       = 0;

minInfo.nMaxIter        = 40;
minInfo.nNumParametersH = nNumParametersH;
minInfo.bRobust         = 0;

debug.bDisp    = 1;
debug.bVerbose = 1;

estimates = MinSSDSL3MotionAffineIllumination( IRef, ICur, ICurMask, template, estimates, minInfo, debug );

H
estimatedH = estimates.HEst
alpha
estimatedAlpha = estimates.alphaEst
beta
estimatedBeta = estimates.betaEst