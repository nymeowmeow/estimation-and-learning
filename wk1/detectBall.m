% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = [ 0.5747, 0.5577, 0.2498];
sig = [0.0042, 0.0025, -0.0043; ...
       0.0025, 0.0024, -0.0033; ...
      -0.0043, -0.0033,  0.0062];
thre = 2.0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
input = im2double(I);
row = size(input,1);
col = size(input,2);
x = reshape(input, row*col, size(input,3));
prob = zeros(size(x, 1), 1);
for i=1:length(prob),
    prob(i,:) = 0.5*(x(i,:) - mu)*pinv(sig)*(x(i,:)-mu)';
end
%prob = mvncdf(x, mu, sig) - mvncdf(mu, mu, sig);
prob = abs(prob)< thre;
bw = reshape(prob, row, col);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[~,idx] = max(numPixels);
segI = false(size(bw));
segI(CC.PixelIdxList{idx}) = true; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
