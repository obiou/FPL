% function estimates = MinSSDSL3Motion( IRef, ICur, template, estimates, minInfo, bDisp )
%
% Tracks a template between two images IRef and ICur by
% sum-of-square differences.
%
% To simplify parameter passing, 3 structures as used as described
% in the following.
%
% Input:
% - IRef: reference image from which the template to track is extracted
% - ICur: current image in which we wish to track the template
% - template: structure containing information on the template to track:
%   * template.HRef: homography for the reference template
%   extractio (typically translation to the top-left corner of the
%   template in IRef)
%   * template.nWidth: template width
%   * template.nHeight: template height
% - estimates:
%   * estimates.HEst: estimated homography
%   * estimates.MEst: estimated blur (in Lie algebra space)
%   * estimates.alphaEst: multiplicative factor in affine illumination model
%   * estimates.betaEst: additive factor in affine illumination model
% - minInfo: structure containing parameters pertaining to the minimisation:
%   * minInfo.nMaxIter: max. number of iterations
%   * minInfo.nNumParametersH: number of parameters to estimate for
%     H (2: translation, 6: affine, 8: homography)
%   * minInfo.nNumParametersM: number of parameters to estimate for
%     M (2: translation, 6: affine, 8: homography)
%   * minInfo.bPreMin: apply homography estimation without blur
%   estimation before the full solve.
%   * minInfo.bRobust: should m-estimators be used (Huber)?
% - debug (optional): 
%   * debug.bDisp: display initial and current template
%   * debug.bVerbose: print RMS
%
% Output:
% - estimates: same structure as input but HEst and MEst have been updated.
%
function estimates = MinSSDSL3MotionSL3BlurAffineIllumination( IRef, ICur, ICurMask, template, estimates, minInfo, debug )

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
alphaEst        = estimates.alphaEst;
betaEst         = estimates.betaEst;

nMaxIter        = minInfo.nMaxIter;
nNumParametersH = minInfo.nNumParametersH;
nNumParametersM = minInfo.nNumParametersM;
bRobust         = minInfo.bRobust;
bPreMinH        = minInfo.bPreMinH;
bPreMinM        = minInfo.bPreMinM;

bDisp           = debug.bDisp;
bVerbose        = debug.bVerbose;

TIRef = warping( IRef, HRef, nWidth, nHeight );

if bDisp
  fig = figure;
  subplot( 1, 2, 1 )
  imshow( uint8( TIRef ) )
  title( 'Reference template' )
end

bestRMS       = Inf;
HEstPrev      = HEst;
HEstCur       = HEst;
HEstBest      = HEst;
MEstPrev      = MEst;
MEstCur       = MEst;
MEstBest      = MEst;
alphaEstPrev  = estimates.alphaEst;
alphaEstCur   = estimates.alphaEst;
alphaEstBest  = estimates.alphaEst;
betaEstPrev   = estimates.betaEst;
betaEstCur    = estimates.betaEst;
betaEstBest   = estimates.betaEst;

dMinNormUpdate = 1e-6;

if bPreMinH
  if bVerbose
    fprintf( 'Pre-minimising with only H.\n' );
  end
  minInfo.nMaxIter = minInfo.bPreMinHMaxIter;
  estimates = MinSSDSL3MotionAffineIllumination( IRef, ICur, ICurMask, template, estimates, minInfo, debug );
  HEstPrev      = estimates.HEst;
  HEstCur       = estimates.HEst;
  HEstBest      = estimates.HEst;
  alphaEstPrev  = estimates.alphaEst;
  alphaEstCur   = estimates.alphaEst;
  alphaEstBest  = estimates.alphaEst;
  betaEstPrev   = estimates.betaEst;
  betaEstCur    = estimates.betaEst;
  betaEstBest   = estimates.betaEst;

  if 0
    estimates.RMS
    RMS = sqrt( mean( ( estimates.TICur(:) - estimates.TIRef(:) ).^2 ) )
    
    figure
    imshow( uint8( estimates.TICur ) )
    
    [newTICur,newTICurMask] = warping( estimates.alphaEst * ICur + estimates.betaEst, template.HRef*estimates.HEst, template.nWidth, template.nHeight ) ;
    [newTIRef,newTIRefMask] = warping( IRef, template.HRef, template.nWidth, template.nHeight );
    
    TIIn = find( newTICurMask == 1 & newTIRefMask == 1 );
        
    max_error = max( abs( newTICur( TIIn ) - estimates.TICur( TIIn ) ) );
    new_RMS = sqrt( mean( ( newTICur( TIIn ) - newTIRef( TIIn) ).^2 ) )
    
    figure
    imshow( uint8( newTICur ) );
    pause
  end
end

if bPreMinM
  if bVerbose
    fprintf( 'Pre-minimising with only M.\n' );
  end
  minInfo.nMaxIter = minInfo.bPreMinMMaxIter;
  estimates = MinSSDNoMotionSL3Blur( IRef, estimates.alphaEst * ICur  + estimates.betaEst, template, estimates, minInfo, debug );
  MEstPrev = estimates.MEst;
  MEstCur  = estimates.MEst;
  MEstBest = estimates.MEst;
end
if bVerbose
  fprintf( 'Full minimisation.\n' );
end
for nIter=1:nMaxIter
  [HEstCur,MEstCur,alphaEstCur,betaEstCur,RMS,dNormUpdate,TIRefEst,TICurEst] = ...
      SSDUpdateSL3MotionSL3BlurAffineIllumination( IRef, ICur, HRef, nWidth, nHeight, HEstPrev, MEstPrev, ...
                                                   nNumParametersH, nNumParametersM, alphaEstPrev, betaEstPrev, bRobust );

  % The RMS correponds to the HEstPrev estimate
  if RMS < bestRMS
    bestRMS      = RMS;
    HEstBest     = HEstPrev;
    MEstBest     = MEstPrev;
    alphaEstBest = alphaEstPrev;
    betaEstBest  = betaEstPrev;
    TIRefBest    = TIRefEst;
    TICurBest    = TICurEst;
  end
   
  HEstPrev     = HEstCur;
  MEstPrev     = MEstCur;
  alphaEstPrev = alphaEstCur;
  betaEstPrev  = betaEstCur;
  
  if dNormUpdate < dMinNormUpdate
    break;
  end
  
  if bVerbose
    fprintf( ' Iter %i, RMS: %f, bestRMS: %f\n', nIter, RMS, bestRMS );
  end
  
  if bDisp
    figure(fig)
    subplot( 1, 2, 1 )
    imshow( uint8( TIRefEst ) )
    title( 'Blurred reference template' )
    subplot( 1, 2, 2 )
    imshow( uint8( TICurEst ) )
    cur_title = sprintf( 'Current template (RMS: %3.2f)', RMS ); 
    title( cur_title )
    drawnow
    %pause
  end
end  

estimates.HEst     = HEstBest;
estimates.MEst     = MEstBest;
estimates.alphaEst = alphaEstBest;
estimates.betaEst  = betaEstBest;
estimates.RMS      = bestRMS;
estimates.TIRef    = TIRefBest;
estimates.TICur    = TICurBest;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( H, M, nNumParametersH, nNumParametersM, alpha, beta )

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
      H = H/det(H)^(1/3);
    case 'trans'
      nNumParametersH = 2;

      H = eye(3);
      H(1,3) = H(1,3)+5;
      H(2,3) = H(2,3);
  end 

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make blur matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  blur_type = 'rot';
  %blur_type = 'zoom';
  %blur_type = 'trans';

  switch blur_type
    case 'rot'
      nNumParametersM = 5;
      
      rot_ax = [0;0;1];
      angle = 15*pi/180;
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
      M(1,3) = M(1,3)+20;
      M(2,3) = M(2,3);
  end 

  alpha = 1.2;
  beta  = 30;

  test( H, M, nNumParametersH, nNumParametersM, alpha, beta );
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
%M = zeros(3);
%H = eye(3);
bNew = 1; bBlur = 1;
fprintf( 'Building simulation images, this can take some time... ' );
if bNew
  % Add blur to avoid aliasing effects due to double warping
  % to build the current image and then current template
  if bBlur
    h = fspecial( 'gaussian', 5, 5 );
    I = imfilter( I, h );
  end

  IRef = I;
  ICur  = 1/alpha * ( blur_warping( I, HRef, M, inv(H)*inv(HRef), nImageWidth, nImageHeight ) - beta );
  
  TIRef = blur_warping( IRef, HRef, M, eye(3), nWidth, nHeight );
  TICur = alpha * warping( ICur, HRef*H, nWidth, nHeight ) + beta;
else
  % Old method, only valid if M or H are neutral values
  IRef = warping( I, HRef*H*inv(HRef), nImageWidth, nImageHeight );
  ICur = blur_warping( I, HRef, M, inv(HRef), nImageWidth, nImageHeight );

  TIRef = blur_warping( IRef, HRef, M, eye(3), nWidth, nHeight );
  TICur = warping( ICur, HRef*H, nWidth, nHeight );
end
ICurMask = ones( size( I ) );
fprintf( 'done.\n' );

absDiff = abs( TIRef(:) - TICur(:) );

fprintf( 'Simulation errors: max: %f, RMS: %f, median: %f.\n', ...
         max( absDiff ), sqrt( mean( absDiff.^2  ) ), median( absDiff ) );

% Setup parameters
template.HRef    = HRef;
template.nWidth  = nWidth;
template.nHeight = nHeight;

estimates.HEst     = eye(3);
estimates.MEst     = zeros(3);
estimates.alphaEst = 1;
estimates.betaEst  = 0;

minInfo.nMaxIter        = 15;
minInfo.nNumParametersH = nNumParametersH;
minInfo.nNumParametersM = nNumParametersM;
minInfo.bPreMinH        = 1;
minInfo.bPreMinHMaxIter = 5;
minInfo.bPreMinM        = 1;
minInfo.bPreMinMMaxIter = 5;
minInfo.bRobust         = 0;

debug.bDisp    = 1;
debug.bVerbose = 1;

estimates = MinSSDSL3MotionSL3BlurAffineIllumination( IRef, ICur, ICurMask, template, estimates, minInfo, debug );

H
estimates.HEst
M
estimates.MEst
alpha
estimates.alphaEst
beta
estimates.betaEst
