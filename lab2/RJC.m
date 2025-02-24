% RJC: Randomized Joint Compatibility Algorithm (version RJC - Algorithm 6)
%
% Input:
%   prediction    - structure with predicted landmarks (must have field 'n')
%   observations  - structure with observations (must have field 'm')
%   compatibility - structure containing the compatibility matrix 'ic'
%
% Output:
%   H - Association hypothesis (vector) with the highest number of pairings
%
% This algorithm combines the idea of RANSAC (random sampling and hypothesis verification)
% with joint compatibility verification (JCBB*) to obtain a robust solution.
function H = RJC(prediction, observations, compatibility)
% Algorithm parameters:
global configuration;
Pfail = 0.01;    % Accepted failure probability
Pgood = 0.8;     % Initial probability of a "good" hypothesis
b = 4;           % Minimum number of pairings for verification (threshold)
%
% Initialization:
i_iter = 1;          % Iteration counter in the while loop
Best_H = [];           % Best hypothesis found (initially empty)
m1 = 1:prediction.n; % Set of candidate indices for association
m2 = 1:prediction.n; % Set from which candidates will be randomly drawn (m*2)
%
% The number of iterations t is calculated as:
t = ceil(log(Pfail) / log(1 - Pgood));
%
% Main loop: repeats until t iterations (which is dynamically updated)
while i_iter <= t
  % Randomly select b indices from m2
  if length(m2) >= b
    m_star2 = m2(randperm(length(m2), b));
  else
    m_star2 = m2;
  end

  % Start the modified joint compatibility verification (JCBB*) with an empty hypothesis
  candidateH = JCBB_star(m1, m_star2, prediction, observations, compatibility, b);

  % Update the best hypothesis if the current one has more pairings
  if pairings(candidateH) > pairings(Best_H)
    Best_H = candidateH;
  end

  % Update the probability of a good hypothesis based on the fraction of pairings
  Pgood = max(Pgood, pairings(Best_H) / observations.m);

  % Recalculate the number of iterations t based on Pgood
  t = ceil(log(Pfail) / log(1 - Pgood));

  i_iter = i_iter + 1;
end

% Return the best hypothesis found
H = Best_H;

configuration.name = 'Randomized Joint Compatibility';



function H = JCBB_star (m1, m_star2, prediction, observations, compatibility, b)
  %
  %-------------------------------------------------------
  global Best;

  Best.H = zeros(1, observations.m);

  JCBB_star_R([], 1, m1, m_star2, prediction, observations, compatibility, b);

  H = Best.H;

  %------------------------------------------------------------------
  % Recursive JCBB* function to test joint compatibility in verification mode
  % for b pairings.
  %
  % H: current hypothesis (association vector)
  % i: observation index to process
  % m1: set of candidate indices for each observation
  % m_star2: random subset drawn from m2 (for verification)
  % b: minimum number of required pairings
  %
  % The function extends H until b pairings are reached; at that
  % moment, the hypothesis is completed using the NN function.
  %------------------------------------------------------------------
function candidate = JCBB_star_R(H, i, m1, m_star2, prediction, observations, compatibility, b)
  global Best;

  % If there are no more observations to process, return the current hypothesis.
  if i > observations.m
    if pairings(H) > pairings(Best.H)
      Best.H = H;
    end
    return;
  end

  % If b pairings have been reached, complete the hypothesis using NN.
  if pairings(H) == b
    candidate = NN(H, i+1, m1, m_star2, prediction, observations, compatibility);
    if pairings(candidate) > pairings(Best.H)
      Best.H = candidate;
    end
    return;
  end

  % For each candidate index (from set m1) for observation i:
  for j = 1:length(m1)
    % Verify the individual compatibility of observation i with candidate j
    if compatibility.ic(i, j)
      % Verify the joint compatibility of the extended hypothesis
      if jointly_compatible(prediction, observations, [H j])
        % Recursive call to process the next observation
        JCBB_star_R([H j], i+1, m1, m_star2, prediction, observations, compatibility, b);

      end
    end
  end
  % Also consider the option of leaving observation i unmatched (0)
  if pairings(H) + observations.m - i > pairings(Best.H) % is it worth continuing?
    JCBB_star_R([H, 0], i+1, m1, m_star2, prediction, observations, compatibility, b);
  end

  %--------------------------------------------------------------------
  % NN function version for JCBB*: completes the hypothesis with the
  % nearest neighbor (NN) for the remaining observations.
function H_out = NN(H, i_iter, m1, m_star2, prediction, observations, compatibility)
  global chi2;

  % Initialize H_out with the current hypothesis
  H_out = H;

  % Process the remaining observations
  for i = i_iter-1:observations.m
    D2min = inf;
    nearest = 0;
    for j = 1:length(m_star2)
      Dij2 = compatibility.d2(i, m_star2(j));
      if Dij2 < D2min
        nearest = m_star2(j);
        D2min = Dij2;
      end
    end
    if D2min <= chi2(2)
      H_out(i) = nearest;
    else
      H_out(i) = 0;
    end
  end

function p = pairings(H)
  if isempty(H)
    p = 0;
  else
    p = length(find(H));
  end