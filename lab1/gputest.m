g_time = tic;

% Define the size of the matrices
N = 5000; % You can change this to any desired size

tic;

% Generate two random NxN matrices
A_gpu = gpuArray.rand(N, N);
B_gpu = gpuArray.rand(N, N);

toc;
tic;

% Multiply the matrices on the GPU
C_gpu = A_gpu * B_gpu;

toc;

tic;
% Transfer the result back to the CPU
C = gather(C_gpu);
toc;

% Display a message (not the full result, since the matrix is large)
disp('Matrix multiplication of NxN matrices completed using gpuArray.');
%disp(['The size of the result matrix is: ', num2str(size(C))]);

toc(g_time);