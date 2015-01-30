function [nodes,tets,faces] = tetgen(V, F, quality, switches)
% TETGEN Interface to the tetgen library.
% 
% [nodes,tets,faces] = tetgen(V, F, quality, switches)
%   V:  3xN set of vertices
%   F:  KxM set of faces, where K is the number of vertices per face
%       (optional, defaults to computing convex hull)
%   quality:  quality of tet mesh (optional, default = 2)
%   switches: tetgen switches     (optional, default = 'Q')
%   nodes: 3xNN set of nodes, NN >= N
%   tets:  4xMM set of tetrahedral elements, MM = # tets
%   faces: 3xPP set of triangular faces (or convex hull if F is empty) PP = # faces
  error('tetgen:mex:error', 'Must compile mex interface')
end