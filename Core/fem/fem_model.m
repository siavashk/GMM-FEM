classdef fem_model < handle
    %FEM_MODEL Generic finite element model
    %
    %   Copyright 2013 C. Antonio Sanchez [antonios@ece.ubc.ca]
    
    %% Private properties
    properties (Access=protected)
        % Nx3 matrix of nodes
        nodes = [];
        
        % Pointer to bv-tree for query
        tree = 0;
        
        % Pointer to surface-mesh tree for query
        smesh_tree = 0;
        
        % indices for faces of surface mesh
        smesh = [];
        
        % Mx1 cell array of elements
        elems = {};
        
        % Elasticity matrix
        elasticity = [];
        
        % Abort stiffness matrix computation if element detected with
        %  determininant less than this
        abortDeterminant = -Inf;
        
        
        % Size of the [i,j,s] vectors used for sparse matrix construction
        sparseVecSize = 0;
        
        % row and column vectors for sparse matrix construction
        sparse_r = [];
        sparse_c = [];
        sparse_s = [];
    end
    
    %% static methods
    methods (Static)
        function fem = createBeam(widths, enum, centre, rotation)
            % Initializes a standard hexahedral beam model
            %
            % createBeam(model, widths, enum, centre, rotation)
            %   Initializes a standard hexahedral beam model
            %
            %   widths:   1x3 lengths of the three sides
            %   enum:     1x3 number of elements along each dimension
            %   centre:   centre of the beam (optional, default = [0,0,0])
            %   rotation: 3x3 rotation matrix for the beam, where rotation
            %             is applied by post-multiplication: y = xR
            %             (optional, default = identity)
            
            if (nargin < 3 || isempty(centre))
                centre = [0, 0, 0];
            end
            if (nargin < 4 || isempty(rotation))
                rotation = eye(3);
            end
            
            dx = widths./enum;
            nsize = enum+1;
            numNodes = prod(nsize);
            numElems = prod(enum);
            
            nnodes = zeros(numNodes,3);
            nelems{numElems,1} = [];  % init
            
            % build nodes
            idx = 1;
            for k=0:enum(3)
                for j=0:enum(2)
                    for i = 0:enum(1)
                        pos = [i*dx(1), j*dx(2), k*dx(3)] - widths/2;
                        pos = pos*(rotation)+centre;
                        
                        nnodes(idx,:) = pos;
                        idx = idx+1;
                    end
                end
            end
            
            % build elements
            idx = 1;
            for k=1:enum(3)
                z = [k; k; k; k; k+1; k+1; k+1; k+1];
                for j=1:enum(2)
                    y = [j; j; j+1; j+1; j; j; j+1; j+1];
                    for i = 1:enum(1)
                        x = [i; i+1; i+1; i; i; i+1; i+1; i];
                        n = sub2ind(nsize,x,y,z);
                        nelems{idx} = fem_hex_element(n');
                        idx = idx+1;
                    end
                end
            end
            
            fem = fem_model(nnodes, nelems);
        end
    end
    
    %% Public methods
    methods
        
        function [model] = fem_model(nodes, elems, D)
            % Constructor
            %
            % model = fem_model(nodes, elems, D)
            %   Creates a new fem_model with supplied nodes, elements and
            %   elasticity matrix
            %
            %   nodes: Nx3 matrix of node positions
            %   elems: MxL matrix of elements, where M is the number of
            %          elements to add, and L is the number of nodes per
            %          element
            %   D:     6x6 elasticity matrix for this model
            
            if (nargin >= 1 && ~isempty(nodes))
                addNode(model, nodes);
            end
            if (nargin >=2 && ~isempty(elems))
                addElement(model, elems);
            end
            if (nargin >=3 && ~isempty(D))
                setElasticity(model, D);
            end
            
        end
        
        function setSurfaceMesh(this, verts, faces)
            this.smesh.vertices = verts;
            this.smesh.faces = faces;
            buildSMeshTree(this);
        end
        
        function buildSMeshTree(this, smesh)
            if (this.smesh_tree > 0)
                this.smesh_tree = 0;
            end
            
            if (nargin < 2 || isempty(smesh))
                smesh = this.smesh;
            else
                this.smesh = smesh;
            end
            
            this.smesh_tree = smesh_bvtree(smesh.vertices', smesh.faces');
        end
        
        function buildBVTree(this)
            % Builds a BVTree containing elements and returns the index
            
            % clear old tree if exists
            if (this.tree > 0)
                this.tree = 0;
            end
            
            elemIdxs = cell(size(this.elems));
            for i=1:numel(this.elems)
                elemIdxs{i} = getNodeIdxs(this.elems{i});
            end
            
            this.tree = bvtree(this.nodes', elemIdxs);
            
        end
        
        function [N] = getNumNodes(this)
            % Returns the number of nodes in this model
            N = size(this.nodes,1);
        end
        
        function [M] = getNumElements(this)
            % Returns the number of elements in this model
            M = length(this.elems);
        end
        
        function set(this, nodes, elems)
            % Sets the model's nodes and elements
            %
            % set(model, nodes, elems)
            %   Populates the model with a set of nodes and elements
            %
            %   nodes:  Nx3 matrix of node positions, N nodes
            %   elems:  MxD matrix of node indices, M is the number of
            %            elements to add, each consisting of D ordered
            %            nodes.  See documentation for fem_element types
            %            for proper node ordering.
            
            this.nodes = [];
            this.elems = {};
            this.sparseVecSize = 0;
            this.sparse_r = [];
            this.sparse_c = [];
            this.sparse_s = [];
            this.tree = 0;
            this.smesh = [];
            this.smesh_tree = 0;
            
            addNode(this, nodes);
            addElement(this, elems);
            
        end
        
        function [idx] = addNode(this, pos)
            % Adds a node or or set of nodes to this finite element model
            %
            % [idx] = addNode(model,pos)
            %    Adds a node or set of nodes with rest position(s) given by
            %    pos (Nx3). Returns the index or set of node indices.
            
            idx = size(this.nodes,1)+1;
            idx = idx:(idx+size(pos,1)-1);
            this.nodes = [this.nodes; pos];
            this.smesh = [];
            this.smesh_tree = 0;
        end
        
        function updateNodes(this, pos, idxs)
            % Updates node locations for this model
            %
            % updateNodes(model, pos, idxs)
            %   Updates node locations for this model
            %   pos:  Kx3 set of positions
            %   idxs: Kx1 indices of nodes to set (optional,
            %         default=1:length(pos,1))
            
            K = size(pos,1);
            if (nargin < 3 || isempty(idxs))
                idxs = 1:K;
            end
            this.nodes(idxs,:) = pos;
            buildBVTree(this)
        end
        
        function [idx, elem] = addElement(this, E)
            % Adds an element or set of elements to this model
            %
            % [idx, nodeIdxs] = addElement(this, nodeIdxs)
            %   Adds an element or set of elements to this model based on
            %   node indices
            %
            % [idx, nodeIdxs] = addElement(this, nodeIdxCells)
            %   Adds an element or set of elements to this model based on
            %   node indices
            %
            % [idx, elem] = addElement(this, E)
            %   Adds an element object or set (array or cell array) to this
            %   model
            %
            %   Inputs:
            %   nodeIdxs MxD matrix of node indices, K is the number of
            %            elements to add, each consisting of D ordered
            %            nodes.
            %
            %            See documentation for fem_element types
            %            for proper node ordering.
            %
            %   nodeIdxCells cell array of 1xD_i vectors indicating the
            %            node indices to use.  Each cell entry can have a
            %            different number of nodes (different element type)
            %
            %   E        single, array, or cell array of fem_element
            %            objects to add
            %
            %   Outputs:
            %   idx      The index of the newly added element(s)
            %   elem     The fem_element object(s)
            
            N = getNumElements(this);
            if (iscell(E))
                % cell array of elements
                idx = ((N+1):(N+numel(E)))';
                M = numel(E);
                elem{M} = [];
                for i=1:M
                    % check if a fem_element object
                    if (isa(E{i},'fem_element'))
                        elem{i} = E{i};
                    else
                        EE = fem_element.create(E{i});
                        elem{i} = EE(1);
                    end
                    D = getNumNodes(elem{i});
                    this.sparseVecSize = this.sparseVecSize + 9*D*D;
                end
            elseif (isa(E,'fem_element'))
                % single element
                idx = N+1;
                elem = {E};
                D = getNumNodes(E);
                this.sparseVecSize = this.sparseVecSize + 9*D*D;
            else
                % node indicies
                elem = fem_element.create(E);
                elem = num2cell(elem);
                M = size(E,1);
                D = size(E,2);
                idx = ((N+1):(N+M))';
                this.sparseVecSize = this.sparseVecSize + 9*M*D*D;
            end
            
            this.elems = [this.elems; elem];
            this.sparse_r = [];
            this.sparse_c = [];
            this.sparse_s = [];
            
            buildBVTree(this);
        end
        
        function n = getNode(this,idx)
            % Returns the location of node idx
            n = this.nodes(idx,:);
        end
        
        function N = getNodes(this)
            % Returns all nodes of this model
            N = this.nodes;
        end
        
        function elem = getElement(this,idx)
            % Returns the element(s) at index idx
            if (numel(idx) == 1)
                elem = this.elems{idx};
            else
                elem = this.elems(idx);
            end
        end
        
        function E = getElements(this)
            % Returns all elements as a cell array of fem_element objects
            E = this.elems;
        end
        
        function D = getElasticity(this)
            % Get the elasticity matrix of this model
            D = this.elasticity;
        end
        
        function treeIdx = getTree(this)
            % Get the bv-tree associated with this model
            treeIdx = this.tree;
        end
        
        function setElasticity(this, D)
            % Sets the 6x6 elasticity matrix for this model
            this.elasticity = D;
        end
        
        function setMaterial(this, mat)
             % Sets the fem_material for this model
            this.elasticity = mat;
        end
        
        function [K, minJ]= getStiffnessMatrix(this, D, symmetric)
            % Computes the global stiffness matrix for linear elastic model
            %
            % K = getStiffnessMatrix(model, D)
            %   Builds the global stiffness matrix.  If an element has a
            %   deformation gradient determinant less than the minimum
            %   allowable (see setAbortJacobian), then stiffness matrix
            %   computations are aborted
            %
            %   D: 6x6 elasticity matrix (optional, defaults to elasticity
            %      stored in the model)
            %      OR
            %      Object of type fem_material, which will compute the
            %      local stiffness matrix block
            %          getStiffnessMatrix(fem_material, point)
            %   symmetric: if 1, enforces symmetry by setting Ksym=(K+K')/2
            %       (optional, default = 0)
            %   K: output stiffness matrix
            %   minJ: minimum computed deformation gradient determinant
            
            if (nargin < 2 || isempty(D))
                D = this.elasticity;
                if (isempty(D))
                    error('Elasticity matrix has not been set');
                end
            end
            
            if (nargin < 3 || isempty(symmetric))
                symmetric = 0;
            end
            
            if (isempty(this.sparse_r))
                [this.sparse_r, this.sparse_c] = getGlobalIndices(this);
            end
            
            this.sparse_s = zeros(this.sparseVecSize,1);
            minJ = Inf;
            
            M = getNumElements(this);
            idx = 1;
            for i=1:M
                [Ke, minJe] = getStiffnessMatrix(this.elems{i}, D, ...
                    this.nodes);
                if (minJe < 0)
                    warning(['Element ',num2str(i),' is inverted']);
                end
                
                if (minJe <= this.abortDeterminant)
                    K = [];
                    minJ = minJe;
                    return;
                else
                    if (minJe < minJ)
                        minJ = minJe;
                    end
                end
                
                ne = numel(Ke);
                this.sparse_s(idx:(idx+ne-1)) = Ke(:);
                idx = idx + ne;
            end
            N3 = 3*getNumNodes(this);
            K = sparse(this.sparse_r, this.sparse_c, this.sparse_s, ...
                N3, N3);
            
            % Enforce symmetry
            if (symmetric == 1)
                K = (K+K')/2;
            end
            
            
        end
        
        function M = getMassMatrix(this)
            % Computes the global mass matrix for unit density
            %
            % M = getMassMatrix(model)
            %   Builds the global mass matrix
            
            if (isempty(this.sparse_r))
                [this.sparse_r, this.sparse_c] = getGlobalIndices(this);
            end
            
            this.sparse_s = zeros(this.sparseVecSize,1);
            
            M = numElements(this);
            idx = 1;
            for i=1:M
                Me = getMassMatrix(this.elems{i}, this.nodes);
                ne = numel(Me);
                this.sparse_s(idx:(idx+ne-1)) = Me(:);
            end
            N3 = 3*numNodes(this);
            M = sparse(this.sparse_r, this.sparse_c, this.sparse_s, ...
                N3, N3);
            
        end
        
        function [idx, elem, S] = findContainingElement(this, x, tol)
            % Finds the element that contains a point
            %
            % [idx, elem, S] = findContainingElement(model, x)
            %   Finds the element that contains the point x, if indeed the
            %   point does fall within the model.  Otherwise, finds the
            %   nearest element.  Returns the index of the
            %   element, the element, and the point's natural coordinates.
            %
            %   x:    Kx3 points in spatial coordinates
            %   idx:  Kx1 element indices if found, 0 if not found
            %   elem: fem_element object(s)
            %   S:    K-length cell array of natural coordinates of each point 
            %         inside the corresponding nearest element
            K = size(x,1);
            
            if (nargin < 3 || isempty(tol) || tol < 0)
                tol = 1e-12;    % should scale by volume of model
            end
            
            [cellIdxs] = intersect_point(this.tree, x', tol);
            
            % for each point, loop through potential elements to find if
            % contained
            idx = zeros(K,1);
            if (nargout > 1)
                elem{K,1} = [];
            end
            if (nargout > 2)
                S{K,1} = [];
            end
            
            numOutside = 0;
            outside = zeros(K,1);
            in = ones(K,1);
            for i=1:K
                eidxs = cellIdxs{i};
                
                inside = 0;
                for j=1:numel(eidxs)
                    e = this.elems{eidxs(j)};
                    X = this.nodes(getNodeIdxs(e),:);
                    [inside, xem] = isInside(e, X, x(i,:));
                    if (inside == 1)
                        idx(i) = eidxs(j);
                        
                        if (nargout > 1)
                            elem{i} = e;
                        end
                        if (nargout > 2)
                            S{i} = xem;
                        end
                        break;
                    end
                end
                if (inside == 0)
                    numOutside = numOutside+1;
                    outside(numOutside) = i;
                    % idx(i) = 0, elem{i} = [], S{i} = []
                end
            end
        end
        
        function [idx, elem, S] = findNearestElement(this, x, tol)
            % Finds the element that contains a point, or the nearest if
            % outside
            %
            % [idx, elem, S] = findNearestElement(model, x)
            %   Finds the element that contains the point x, if indeed the
            %   point does fall within the model.  Otherwise, finds the
            %   nearest element.  Returns the index of the
            %   element, the element, and the point's natural coordinates.
            %
            %   x:    Kx3 points in spatial coordinates
            %   idx:  Kx1 element indices if found, 0 if not found
            %   elem: fem_element object(s)
            %   S:    K-length cell array of natural coordinates of each point 
            %         inside the corresponding nearest element
            K = size(x,1);
            
            if (nargin < 3 || isempty(tol) || tol < 0)
                tol = 1e-12;    % should scale by volume of model
            end
            
            [cellIdxs] = intersect_point(this.tree, x', tol);
            
            % for each point, loop through potential elements to find if
            % contained
            idx = zeros(K,1);
            if (nargout > 1)
                elem{K,1} = [];
            end
            if (nargout > 2)
                S{K,1} = [];
            end
            
            numOutside = 0;
            outside = zeros(K,1);
            in = ones(K,1);
            for i=1:K
                eidxs = cellIdxs{i};
                
                inside = 0;
                for j=1:numel(eidxs)
                    e = this.elems{eidxs(j)};
                    X = this.nodes(getNodeIdxs(e),:);
                    [inside, xem] = isInside(e, X, x(i,:));
                    if (inside == 1)
                        idx(i) = eidxs(j);
                        
                        if (nargout > 1)
                            elem{i} = e;
                        end
                        if (nargout > 2)
                            S{i} = xem;
                        end
                        break;
                    end
                end
                if (inside == 0)
                    numOutside = numOutside+1;
                    outside(numOutside) = i;
                    % idx(i) = 0, elem{i} = [], S{i} = []
                end
            end
            
            outside = outside(1:numOutside);
            
            % deal with points outside
            if (numOutside > 0) 
                warning(['There are points outside the mesh']);
            end
        end
        
        function [idx, elem, S] = findNearestElementSeq(this, x, tol)
            % Finds the element that contains a point, or the neasest if
            % outside
            %
            % [idx, elem, N, in, XEM] = findNearestElementSeq(model, x)
            %   Finds the element that contains the point x, if indeed the
            %   point does fall within the model.  Otherwise, finds the
            %   nearest element.  Returns the index of the
            %   element, the element, and the point's natural coordinates.
            %
            %   x:    Kx3 points in spatial coordinates
            %   idx:  Kx1 element indices if found, 0 if not found
            %   elem: fem_element object(s)
            %   S:    K-length cell array of natural coordinates of each point 
            %         inside the corresponding nearest element
            K = size(x,1);
            
            if (nargin < 3 || isempty(tol) || tol < 0)
                tol = 1e-12;    % should scale by volume of model
            end
                        
            % for each point, loop through potential elements to find if
            % contained
            idx = zeros(K,1);
            if (nargout > 1)
                elem{K,1} = [];
            end
            if (nargout > 2)
                S{K,1} = [];
            end
            
            numOutside = 0;
            outside = zeros(K,1);
            
            lasteidx = 1;
            lastelem = this.elems{lasteidx};
            
            for i=1:K
                X = this.nodes(getNodeIdxs(lastelem),:);
                [inside, xem] = isInside(lastelem, X, x(i,:));
                if (inside == 1)
                    idx(i) = lasteidx;
                    if (nargout > 1)
                        elem{i} = lastelem;
                    end
                    if (nargout > 2)
                        S{i} = xem;
                    end
                    break;
                else 
                    [cellIdxs] = intersect_point(this.tree, x(i,:)', tol);
                    eidxs = cellIdxs{1};    
                    inside = 0;
                    for j=1:numel(eidxs)
                        e = this.elems{eidxs(j)};
                        X = this.nodes(getNodeIdxs(e),:);
                        [inside, xem] = isInside(e, X, x(i,:));
                        if (inside == 1)
                            idx(i) = eidxs(j);
                            if (nargout > 1)
                                elem{i} = e;
                            end
                            if (nargout > 2)
                                S{i} = xem;
                            end
                            lastelem = e;
                            lasteidx = eidxs(j);
                            break;
                        end
                    end
                    if (inside == 0)
                        numOutside = numOutside+1;
                        outside(numOutside) = i;
                    end        
                end
            
            end
            
            outside = outside(1:numOutside);
            
            % deal with points outside
            if (numOutside > 0) 
                if (this.smesh_tree ~= 0)
                    x = x(outside,:);
                    [idx, nearest] = nearest_polygon(this.smesh_tree, x');
                    cellIdxs = intersect_point(this.tree, nearest, tol);

                    for i=1:numOutside
                        eidxs = cellIdxs{i};
                        oidx = outside(i);
                        if (numel(eidxs) > 0)
                            eidx = eidxs(1);
                            e = this.elems{eidx};
                            X = this.nodes(getNodeIdxs(e),:);
                            xem = getNaturalCoords(e, X, x(i,:));
                            
                            idx(oidx) = eidx;
                            if (nargout > 1)
                                elem{oidx} = e;
                            end
                            if (nargout > 2)
                                S{oidx} = xem;
                            end
                        else
                            fprintf('Point not nearest anything...\n');
                        end
                    end
                else
                    warning(['Surface-mesh not defined; cannot determine ',...
                        'nearest elements for points outside']);
                end
            end
        end
        
        
        function [idx, dist] = findNearestNode(this, x)
            % Finds the node nearest to x
            %
            % [idx, dist] = findNearestNode(model, x)
            %   Finds the node nearest to location x
            %
            %   x:    Mx3 input location(s)
            %   idx:  index of resulting nearest node
            %   dist: distance to nearest node
            
            N = getNumNodes(this);
            M = size(x,1);
            
            x = permute(x,[3 2 1]);
            diff = repmat(this.nodes,[1,1,M])-repmat(x,[N,1,1]);
            diff = sum(diff.*diff,2);
            [dist, idx] = min(diff,[],1);
            idx = idx(:);
            dist = sqrt(dist(:));
        end
        
        function [ P, in, dP ] = getInterpolationMatrix( this, X )
            % Determines the interpolation matrix for embedded positions X
            %
            % [P, in, dP] = getInterpolationMatrix(model, X)
            %   Determines the FEM interpolation matrix (and optionally its
            %   gradient) from the points in X, such that X = P*nodes.  
            %   This can be used for tracking the positions of points that fall 
            %   inside the finite element model.
            %
            %   X:  Kx3 list of positions in spatial coordinates
            %   nodes: Nx3 list of node positions
            %   P:  KxN interpolation matrix
            %   in: Kx1 vector, 1 indicating the point is contained by the
            %           model, 0 if outside
            %   dP: 3KxN gradient matrix
            
            M = size(X,1);
            [~, elem, S] = findContainingElement(this, X);
            
            ntot = 0;
            % count sizes
            for i=1:M
                if (~isempty(elem{i}))
                    ntot = ntot+getNumNodes(elem{i});
                end
            end
            
            r = zeros(ntot,1);
            c = zeros(ntot,1);
            s = zeros(ntot,1);
            
            if (nargout > 1)
                in = zeros(M, 1);
            end
            if (nargout > 2)
                ds = zeros(3*ntot,1);
            end
            idx = 1;
            for i=1:M
                if (~isempty(elem{i}))
                    nodeIdxs = getNodeIdxs(elem{i});
                    xem = S{i};
                    
                    ne = numel(nodeIdxs);
                    idxs=idx:(idx+ne-1);
                    
                    N = getN(elem{i}, xem(1), xem(2), xem(3));
                    r(idxs) = i;
                    c(idxs) = nodeIdxs;
                    s(idxs) = N;
                    
                    if (nargout > 1)
                        in(i) = 1;
                    end
                    if (nargout > 2)
                        dNds = getdN(elem{i}, xem(1), xem(2), xem(3));
                        XX = this.nodes(getNodeIdxs(elem{i}),:);
                        J = getJ(elem{i}, dNds, XX);
                        dNdx = getShapeGrad(elem{i}, dNds, J);
                        ds(idxs) = dNdx(1,:);
                        ds(idxs+ntot) = dNdx(2,:);
                        ds(idxs+2*ntot) = dNdx(3,:);
                    end

                    idx = idx+ne;
                end
            end
            
            N = getNumNodes(this);
            P = sparse(r,c,s,M,N);
            if (nargout > 2)
                br = 3*(r-1);
                dr = [br+1;br+2;br+3];
                dc = [c;c;c];
                dP = sparse(dr,dc,ds,3*M,N);
            end

        end
        
        function [ P, in ] = getInterpolationMatrixSeq( this, X )
            % Determines the interpolation matrix for embedded positions X
            %
            % P = getInterpolationMatrix(model, X)
            %   Determines the FEM interpolation matrix from the points in
            %   X, such that X = P*nodes.  This can be used for tracking
            %   points that fall inside the finite element model.
            %
            %   X:  Kx3 list of positions in spatial coordinates
            %   nodes: Nx3 list of node positions
            %   P:  KxN interpolation matrix
            
            M = size(X,1);
            %[~, elem, N] = findContainingElement(this, X);
            [~, elem, S] = findNearestElementSeq(this, X);
            
            ntot = 0;
            % count sizes
            for i=1:M
                if (~isempty(elem{i}))
                    ntot = ntot+getNumNodes(elem{i});
                end
            end
            
            r = zeros(ntot,1);
            c = zeros(ntot,1);
            s = zeros(ntot,1);
            
            if (nargout > 1)
                in = zeros(M, 1);
            end
            if (nargout > 2)
                ds = zeros(3*ntot,1);
            end
            
            idx = 1;
            for i=1:M
                if (~isempty(elem{i}))
                    nodeIdxs = getNodeIdxs(elem{i});
                    xem = S{i};
                    N = getN(elem{i}, xem(1), xem(2), xem(3));
                    
                    ne = numel(nodeIdxs);
                    idxs=idx:(idx+ne-1);
                    r(idxs) = i;
                    c(idxs) = nodeIdxs;
                    s(idxs) = N;
                    
                    if (nargout > 1)
                        in(i) = 1;
                    end
                    if (nargout > 2)
                        dNds = getdN(elem{i}, xem(1), xem(2), xem(3));
                        XX = this.nodes(getNodeIdxs(elem{i}),:);
                        J = getJ(elem{i}, dNds, XX);
                        dNdx = getShapeGrad(elem{i}, dNds, J);
                        ds(idxs) = dNdx(1,:);
                        ds(idxs+ntot) = dNdx(2,:);
                        ds(idxs+2*ntot) = dNdx(3,:);
                    end
                    
                    idx = idx+ne;
                end
            end
            
            N = getNumNodes(this);
            P = sparse(r,c,s,M,N);
            if (nargout > 2)
                br = 3*(r-1);
                dr = [br+1;br+2;br+3];
                dc = [c;c;c];
                dP = sparse(dr,dc,ds,3*M,N);
            end
        end
        
        
        function setAbortJacobian(this, detJ)
            % SETABORTJACOBIAN sets minimum allowable Jacobian determinant
            %
            % setAbortJacobian(model, detJ)
            %   Sets the minimum allowable deformation determinant before
            %   aborting stiffness matrix computations.  Set detJ to 0 to
            %   abort on inverted elements.  By default, detJ set set to
            %   -Inf, meaning it will never abort
            
            this.abortDeterminant = detJ;
        end
        
        function delete(this)
            if (~isempty(this.smesh_tree) && this.smesh_tree ~= 0)
                delete(this.smesh_tree)
            end
            if (~isempty(this.tree) && this.tree ~= 0)
                delete(this.tree)
            end
            if (~isempty(this.elasticity) && isa(this.elasticity,'fem_material'))
                delete(this.elasticity)
            end
            for i=1:numel(this.elems)
                delete(this.elems{i})
            end
            this.elems = {};
        end
        
    end
    
    methods (Access = protected)
        function [r, c] = getElementGlobalIndices(~, elem)
            % Returns the global matrix indices used by the element
            %
            % [r, c] = getElementGlobalIndices(model, elem)
            %   Determines the global matrix rows (r) and columns (c)
            %   filled by the supplied element.  These arrays are
            %   constructed such that we can fill the sparse mass and
            %   stiffness matrices.
            %
            % r: Mx1 column vector of row indices, where M is the total
            %    number of entries in the element mass/stiffness matrix
            % c: Mx1 column vector of column indices
            
            nodeIdxs = getNodeIdxs(elem);
            N = 3*numel(nodeIdxs);
            A = [3*nodeIdxs-2; 3*nodeIdxs-1; 3*nodeIdxs];
            A = repmat(A(:),[1,N]);
            r = reshape(A,[],1);
            c = reshape(A',[],1);
        end
        
        function [r, c] = getGlobalIndices(this)
            % Returns the index vectors used for global matrix construction
            %
            % [r, c] = getGlobalIndices(model)
            %   Determines the global matrix rows (r) and columns (c)
            %   filled by this model's elements.  These arrays become the
            %   row/column indices for building sparse mass and stiffness
            %   matrices
            %
            % r: Nx1 column vector of row indices, where N is the total
            %    number of entries of all combined element mass/stiffness
            %    matrices
            % c: Nx1 column vector of column indices
            
            M = getNumElements(this);
            idx = 1;
            r = zeros(this.sparseVecSize,1);
            c = r;
            for i=1:M
                [er, ec] = getElementGlobalIndices(this, this.elems{i});
                ne = numel(er);
                r(idx:(idx+ne-1)) = er;
                c(idx:(idx+ne-1)) = ec;
                idx = idx+ne;
            end
            
        end
    end
    
end

