classdef fem_material_spatial_linear_twomaterials < fem_material_spatial_linear
    %FEM_MATERIAL Generic finite element material
    %
    %   Copyright 2014 C. Antonio Sanchez [antonios@ece.ubc.ca]
    
     %% Private properties
    properties (Access=protected)
        % Young's modulus
        myEIn = [];
        myEOut = [];
        
        % Poisson's ratio
        myNuIn = [];
        myNuOut = [];
        
        % Stored 'D' matrix
        DIn = [];
        DOut = [];
        
        % surface mesh for test whether is inside
        smesh = [];
        smesh_fem = [];        
        
    end
    
    
    %% Public methods
    methods
        
        % Constructor
        function mat = fem_material_spatial_linear_twomaterials(...
                EIn, nuIn, EOut, nuOut, verts, faces)
            
            mat.myEIn = EIn;
            mat.myNuIn = nuIn;
            mat.DIn = fem_material_linear.getElasticity(EIn, nuIn);
            
            mat.myEOut = EOut;
            mat.myNuOut = nuOut;
            mat.DOut = fem_material_linear.getElasticity(EOut, nuOut);
            
            setSurfaceMesh(mat, verts, faces);
            
        end
        
        function setSurfaceMesh(this, verts, faces)
            this.smesh.vertices = verts;
            this.smesh.faces = faces;
            buildSurfaceFem(this, this.smesh);
        end
        
        function buildSurfaceFem(this, smesh)
            if (~isempty(this.smesh_fem))
                delete(this.smesh_fem);
            end
            
            if (nargin < 2 || isempty(smesh))
                smesh = this.smesh;
            else
                this.smesh = smesh;
            end
            
            [nodes, elems] = tetgen(smesh.vertices', smesh.faces',[],'');
            this.smesh_fem = fem_model(nodes', elems');
        end
            
        function [in] = isInside(this, pnts)
            idx = findContainingElement(this.smesh_fem, pnts);
            in = (idx > 0);
        end
        
        function D = getElasticity(this, pos)
            % Returns the spatially-dependent 6x6 linear Hooke's law matrix
            % material: this fem_material
            % pos:      1x3 spatial location
            
            % test if inside or outside
            in = isInside(this, pos);
            if (in(1) > 0) 
                D = this.DIn;    
            else
                D = this.DOut;
            end

        end
        
        function delete(this)
            if (~isempty(this.smesh_fem))
                delete(this.smesh_fem);
            end
        end
    end
    
end

