classdef fem_material_spatial_linear < fem_material
    %FEM_MATERIAL Generic finite element material
    %
    %   Copyright 2014 C. Antonio Sanchez [antonios@ece.ubc.ca]
    
    
    %% Abstract methods
    methods % (Abstract) Abstract not properly supported by GNU Octave
        
        function getElasticity(material, pos)
        % Returns the spatially-dependent 6x6 linear Hooke's law matrix
        % material: this fem_material
        % pos:      3D spatial location
            D = [];
        end
    end
    
    %% Public methods
    methods
        
        function [K, minJ] = getStiffnessMatrix(this, elem, nodeArray)
            % Returns the stiffness matrix for the supplied element
            %
            % [K, minJ] = getStiffnessMatrix(mat, elem, nodeArray)
            %   Computes the element stiffness matrix.  Optionally returns
            %   the minimum computed deformation gradient determinant,
            %   useful for determining if an element is inverted.
            %
            %   elem: fem_element object to use for computing stiffness
            %         matrix block
            %
            %   nodeArray: the Nx3 matrix respresenting the set of ALL
            %       nodes, not just the ones belonging to this element
            
            [ipnts, w] = getIPnts(elem);
            n = getNumNodes(elem);
            K = zeros([3*n,3*n]);
            minJ = inf;
            X = nodeArray(getNodeIdxs(elem),:);
                
            % linear material
            for i=1:length(w)
                dNds = getdN(elem, ipnts(i,1), ipnts(i,2), ipnts(i,3));
                J = getJ(elem,dNds,X);
                detJ = det(J);
                if (detJ < minJ)
                    minJ = detJ;
                end
                dNdx = getShapeGrad(elem,dNds, J);
                B = getB(elem,dNdx);
             
                pos = getSpatialCoords(elem, X,...
                    ipnts(i,1), ipnts(i,2), ipnts(i,3));
                
                Dlocal = getElasticity(this, pos);
             
                
                K = K + w(i)*detJ*(B'*Dlocal*B);
            end
            
        end
           
    end
    
end

