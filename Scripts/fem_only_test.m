clear; clc; close all;
add_bcpd_paths;

%% Read in the label maps
[X,fX] = read_ply([root 'Data/US_model.ply']);
[Y,fY] = read_ply([root 'Data/MR_model.ply']);

% The Slicer model has too many vertices and faces. I need to downsample
% it to use it.
numberOfFaces = 1800;

pX.faces = fX;
pX.vertices = X;

pY.faces = fY;
pY.vertices = Y;

nfx = reducepatch(pX,numberOfFaces);
nfy = reducepatch(pY,numberOfFaces);

% Hand-picked landmarks by me and verified by a radiologist NxD format
landmark_MR = [4.64 -70.300 -1.825; 4.579 -68.214 -3.795; -2.736 -58.524 -7.623; -1.984 -60.930 -7.623];
landmark_US = [5.281 -69.204 -1.825; 4.697 -66.087 -3.795; -3.112 -56.945 -7.623; -2.510 -58.975 -7.623];

%% Non-rigid registration parameters
w=0.00; errtol=1e-4; maxiters=500; sigma2=10;
beta=0.12; E=4.8; nu=0.49;

tic;
[TY, ~, ~, ~, ~, newSigma2, ~, fem, u, ~] = cpd_fem_only(nfx.vertices, nfy.vertices, nfy.faces, w, errtol, maxiters, eye(3), [0;0;0], 1.0, sigma2, beta, E, nu, [], [], []);
clc;
time=toc; fprintf('FEM registration time is %f seconds\n', time);

% Interpolate the FEM to find the displacement field in MR landmark locations
tic;
Phi = getInterpolationMatrix(fem, landmark_MR);
time=toc; fprintf('FEM interpolation time is %f seconds\n', time);

% Warp MR landmarks
landmark_warped_MR = landmark_MR + Phi*u;

% Calculate the TRE
error_before_reg = sqrt(sum((landmark_MR - landmark_US).*(landmark_MR - landmark_US),2));
error_after_reg = sqrt(sum((landmark_US - landmark_warped_MR).*(landmark_US - landmark_warped_MR),2));

for i=1:size(error_before_reg,1)
    fprintf('TRE before: %f mm.', error_before_reg(i));
    fprintf(' TRE after: %f mm.\n', error_after_reg(i));
end
