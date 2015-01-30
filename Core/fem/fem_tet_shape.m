function [ Phi, V, B ] = fem_tet_shape( X, E )
%FEM_TET_SHAPE Computes shape functions of tetrahedral elements 
%
%   Computes tetrahedral shape functions with nodes given in X, and 
%   elements described by integer indices in E.  Node order is CW
%   on first face with outward normal.
%
%   The output, Phi is D xsize(E), where the first dimension corresponds
%   to coefficients of the shape functions, which are of the form:
%   phi(x,y,z) = a + b*x + c*y + d*z

    D = size(X,2);
    NE = size(E,1);
    DE = size(E,2); % must be D+1 for simplex
    
    Phi = zeros(D+1,DE,NE);
    V = zeros(NE, 1);
    
    pairs = nchoosek(1:D,2);
    NP = size(pairs,1);
    
    B = zeros(D+NP,D,NE);
    
    OD = ones(DE,1);
    for i=1:NE
        A = [OD X(E(i,:),:)];
        V(i) = det(A)/6;    % volume
        C = cofactor(A);
        for j=1:DE
            B(1:D, ((j-1)*D+1):j*D, i) = diag(C(j,2:end))/(6*V(i));
            for k=1:NP
                B(D+k, (j-1)*D+pairs(k,1), i) = C(j,pairs(k,2)+1)/(6*V(i));
                B(D+k, (j-1)*D+pairs(k,2), i) = C(j,pairs(k,1)+1)/(6*V(i));
            end
        end
        Phi(:,:,i) = C/(6*V(i));
    end

end

