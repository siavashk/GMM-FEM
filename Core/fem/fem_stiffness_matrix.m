function [ K ] = fem_stiffness_matrix( nodes, elems, D )
%FEM_STIFFNESS computes the stiffness matrix K 
%
%   K = fem_stiffness_matrix(nodes, elems, D)
%       Computes the global stiffness matrix K given
%
%       nodes: an Nx3 matrix of node locations
%       elems: an Mx1 cell array of fem_element objects
%       D:  a 6x6 elasticity matrix

    if (size(nodes,2) > 1)
        N = 3*size(nodes,1);
    else
        N = size(nodes,1);
    end
    K = sparse(N,N);
    
    for i=1:length(elems)
        Kij = getStiffnessMatrix(elems{i}, D, nodes);
        nodeIdxs = getNodeIdxs(elems{i});
        Kidx = 3*(nodeIdxs-1);
        
        Kidx = [Kidx+1;
                Kidx+2;
                Kidx+3];
        K(Kidx,Kidx) = K(Kidx,Kidx) + Kij;
    end


end

