classdef fem_material_linear < fem_material
    %FEM_MATERIAL Generic finite element material
    %
    %   Copyright 2014 C. Antonio Sanchez [antonios@ece.ubc.ca]
    
     %% Private properties
    properties (Access=protected)
        % Young's modulus
        myE = [];
        % Poisson's ratio
        myNu = [];
        % Stored 'D' matrix
        D = [];
    end
    
    %% Static methods
    methods(Static)
        function D = getElasticity(E, nu)
            % Computes the 6x6 linear elasticity matrix given Young's
            % modulus and Poisson's ratio
            
            lambda = E*nu/((1+nu)*(1-2*nu));
            mu = E/(2*(1+nu));

            D = mu*eye(6);
            D(1:3,1:3) = lambda+2*mu*eye(3);
        end
    end
    
    %% Public methods
    methods
        
        function mat = fem_material_linear(E, nu)
            % Constructor
            %
            % mat = fem_material_linear(E, nu)
            %   Creates a new fem_material_linear
            %
            %   E: Young's modulus
            %   nu: Poisson's ratio
            mat.myE = E;
            mat.myNu = nu;
            mat.D = fem_material_linear.getElasticity(E, nu);
        end
        
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
                dNds = getdNds(elem, ipnts(i,1),ipnts(i,2),ipnts(i,3));
                J = getJ(elem,dNds,X);
                detJ = det(J);
                if (detJ < minJ)
                    minJ = detJ;
                end
                dNdx = getShapeGrad(elem,dNds, J);
                B = getB(elem,dNdx);
                K = K + w(i)*detJ*(B'*this.D*B);
            end
            
        end
           
    end
    
end

