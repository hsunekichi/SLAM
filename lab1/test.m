% Define the size of the matrices
N = 10000; % You can change this to any desired size

tic;

% Generate two random NxN matrices
A = rand(N, N);
B = rand(N, N);

toc;
tic;

% Multiply the matrices
C = A * B;

% Print exec time
toc;

% Display the result
disp('The product of matrices A and B is:');
%disp(C);
