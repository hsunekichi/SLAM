function H = SINGLES (prediction, observations, compatibility)
%-------------------------------------------------------
% University of Zaragoza
% Authors:  J. Neira, J. Tardos
%-------------------------------------------------------
%-------------------------------------------------------
global chi2;
global configuration;

H = zeros(1, observations.m);

% ----- 1st version -----
% You have observations.m observations, and prediction.n
% predicted features.
%
% For every observation i, check whether it has only one neighbour,
% say feature j, and whether that feature j  has only that one neighbour
% observation i.  If so, H(i) = j.
%
% You will need to check the compatibility.ic matrix
% for this:
%
% compatibility.ic(i,j) = 1 if observation i is a neighbour of
% feature j.

% for i = 1:observations.m
%   neighbors_i = find(compatibility.ic(i, :) == 1);
%   if length(neighbors_i) == 1
%     j = neighbors_i(1);
%     neighbors_j = find(compatibility.ic(:, j) == 1);
%     if length(neighbors_j) == 1 && neighbors_j(1) == i
%       H(i) = j;
%     end
%   end
% end

% ----- 2nd version -----
% % Precompute the number of neighbors for each observation and feature
% num_neighbors_obs = sum(compatibility.ic, 2);
% num_neighbors_feat = sum(compatibility.ic, 1);

% for i = 1:observations.m
%   if num_neighbors_obs(i) == 1
%     j = find(compatibility.ic(i, :) == 1);
%     if num_neighbors_feat(j) == 1
%       H(i) = j;
%     end
%   end
% end

% ----- 3rd version -----
% Precompute the number of neighbors for each observation and feature
num_neighbors_obs = sum(compatibility.ic, 2);
num_neighbors_feat = sum(compatibility.ic, 1);

% Find indices where observations have exactly one neighbor
single_obs_indices = find(num_neighbors_obs == 1);

% Find the corresponding feature indices
single_feat_indices = arrayfun(@(i) find(compatibility.ic(i, :) == 1), single_obs_indices);

% Check if those features also have exactly one neighbor
valid_obs_indices = single_obs_indices(num_neighbors_feat(single_feat_indices) == 1);
valid_feat_indices = single_feat_indices(num_neighbors_feat(single_feat_indices) == 1);

% Assign valid matches to H
H(valid_obs_indices) = valid_feat_indices;

configuration.name = 'SINGLES';
