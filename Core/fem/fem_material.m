classdef fem_material < handle
    %FEM_MATERIAL Generic finite element material
    %
    %   Copyright 2014 C. Antonio Sanchez [antonios@ece.ubc.ca]
    
    %% Public methods
    methods % (Abstract) Abstract not properly supported by GNU Octave
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
            K = [];
            minJ = -1;    
        end
           
    end
    
    methods
        function delete(this)
            % nothing
        end
    end
    
end

