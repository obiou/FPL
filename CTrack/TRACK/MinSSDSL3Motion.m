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
%   extractio (typically translation to the top-left corner of the
%   template in IRef)
%   * template.nWidth: template width
%   * template.nHeight: template height
% - estimates: structure containing the estimated homography in "estimates.HEst"
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
function estimates = MinSSDSL3Motion( IRef, ICur, template, estimates, minInfo, debug )

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

bestRMS  = Inf;
HEstPrev = HEst;
HEstCur  = HEst;
HEstBest = HEst;

dMinNormUpdate = 1e-8;

for nIter=1:nMaxIter
  [HEstCur,RMS,dNormUpdate,TICur] = SSDUpdateSL3Motion( TIRef, ICur, HRef, HEstPrev, nNumParametersH, bRobust );

  % The RMS correponds to the HEstPrev estimate
  if RMS < bestRMS
    bestRMS   = RMS;
    HEstBest  = HEstPrev;
    TICurBest = TICur;
  end
   
  HEstPrev = HEstCur;
  
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

estimates.HEst  = HEstBest;
estimates.RMS   = bestRMS;
estimates.TICur = TICurBest;
estimates.TIRef = TIRef;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test( H, nNumParametersH, minInfo )

if nargin == 0
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Make matrix
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %sl3_type = 'rot';
  %sl3_type = 'zoom';
  %sl3_type = 'trans';
  sl3_type = 'trans_inplanerot';
  
  switch sl3_type
    case 'rot'
      nNumParametersH = 5;
      minInfo.nMaxIter        = 30;
      
      rot_ax = [0;0;1];
      angle = 5*pi/180;
      M = angle*skew(rot_ax);
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
    case 'zoom'
      nNumParametersH = 8;
      minInfo.nMaxIter        = 30;
      
      M = zeros(3,3);
      M(1,1) = M(1,1)-0.05;
      M(2,2) = M(2,2)-0.1;
      N = M*[150;200;0];
      M(1,3) = M(1,3)-N(1);
      M(2,3) = M(2,3)-N(2);
      H = expm( M );
    case 'trans'
      nNumParametersH = 2;
      minInfo.nMaxIter        = 30;

      H = eye(3);
      H(1,3) = H(1,3)-15;
      H(2,3) = H(2,3)+5;
    case 'trans_inplanerot';
      nNumParametersH = 3;
      minInfo.nMaxIter        = 50;

      angle = 15*pi/180;
      R = [cos(angle) sin(angle);
           -sin(angle) cos(angle) ];
      H = [R [-15; 10];
           0 0 1]
  end 

  test( H, nNumParametersH, minInfo );
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

IRef = warping( I, HRef*H*inv(HRef), size(I,2), size(I,1) );
ICur = I;

% Setup parameters
template.HRef           = HRef;
template.nWidth         = nWidth;
template.nHeight        = nHeight;

estimates.HEst          = eye(3);

minInfo.nNumParametersH = nNumParametersH;
minInfo.bRobust         = 0;

debug.bDisp    = 1;
debug.bVerbose = 1;

estimates = MinSSDSL3Motion( IRef, ICur, template, estimates, minInfo, debug );

H
estimatedH = estimates.HEst
