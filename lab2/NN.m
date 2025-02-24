%-------------------------------------------------------
function H = NN (prediction, observations, compatibility)
%-------------------------------------------------------
% University of Zaragoza
% Authors:  J. Neira, J. Tardos
%-------------------------------------------------------
global chi2;
global configuration;

% ----- 1st version -----
% for i = 1:observations.m,
%   D2min = compatibility.d2 (i, 1);
%   nearest = 1;
%   for j = 2:prediction.n,
%     Dij2 = compatibility.d2 (i, j);
%     if Dij2 < D2min
%       nearest = j;
%       D2min = Dij2;
%     end
%   end
%   if D2min <= chi2(2)
%     H(i) = nearest;
%   else
%     H(i) = 0;
%   end
% end

% ----- 2nd version -----
% Preallocate H with zeros
H = zeros(1, observations.m);

% Find the minimum distance and corresponding index for each observation
[D2min, nearest] = min(compatibility.d2, [], 2);

% Assign nearest neighbors based on chi2 threshold
H(D2min <= chi2(2)) = nearest(D2min <= chi2(2));

configuration.name = 'NEAREST NEIGHBOUR';