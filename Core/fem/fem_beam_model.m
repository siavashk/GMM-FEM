classdef fem_beam_model < fem_model
    %FEM_BEAM_MODEL Finite element beam model (regular hex grid)
    %
    %   Copyright 2013 C. Antonio Sanchez [antonios@ece.ubc.ca]
    
    properties(Access = private)
        
        rotation = eye(3);
        centre = zeros(3,1);
        widths = [];
        enum = [];
        
    end
    
    methods
        
        function model = fem_beam_model(widths, enum, centre, rotation)
            % Constructor, builds a standard hexahedral beam
            %
            % model = fem_beam_model(widths, num, centre, rotation)
            %   Builds a standard hexahedral beam
            %
            %   widths:   1x3 lengths of the three sides
            %   enum:     1x3 number of elements along each dimension
            %   centre: centre of the beam (optional, default = [0, 0, 0])
            %   rotation: 3x3 rotation matrix for the beam, where rotation
            %             is applied by post-multiplication: y = xR
            %             (optional, default = identity)
            if (nargin < 3 || isempty(centre) )
                centre = [0 0 0];
            end
            if (nargin < 4 || isempty(rotation) )
                rotation = eye(3);
            end
            
            set(model, widths, enum, centre, rotation);
            
        end
        
        function set(this, widths, enum, centre, rotation)
            % Initializes a standard hexahedral beam model
            %
            % set(model, widths, enum, centre, rotation)
            %   Initializes a standard hexahedral beam model
            %
            %   widths:   1x3 lengths of the three sides
            %   enum:     1x3 number of elements along each dimension
            %   centre:   centre of the beam (optional, default = [0,0,0])
            %   rotation: 3x3 rotation matrix for the beam, where rotation
            %             is applied by post-multiplication: y = xR
            %             (optional, default = identity)
            
            this.widths = widths;
            this.enum = enum;
            
            if (nargin < 4 || isempty(centre))
                centre = [0, 0, 0];
            end
            if (nargin < 5 || isempty(rotation))
                rotation = eye(3);
            end
            
            this.centre = centre;
            this.rotation = rotation;
            
            dx = widths./enum;
            nsize = enum+1;
            numNodes = prod(nsize);
            numElems = prod(enum);
            
            this.nodes = zeros(numNodes,3);
            this.elems{numElems,1} = [];  % init
            
            % build nodes
            idx = 1;
            for k=0:enum(3)
                for j=0:enum(2)
                    for i = 0:enum(1)
                        pos = [i*dx(1), j*dx(2), k*dx(3)] - widths/2;
                        pos = pos*(rotation)+centre;
                        
                        this.nodes(idx,:) = pos;
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
                        this.elems{idx} = fem_hex_element(n');
                        idx = idx+1;
                    end
                end
            end
            
            % prepare sparse info
            this.sparseVecSize = 9*numElems*64;
            this.sparse_r = [];
            this.sparse_c = [];
            this.sparse_s = [];
            
        end
        
        function setCenter(this, centre)
            % Moves the centre of the beam to the following location
            
            N = getNumNodes(this);
            this.nodes = this.nodes + repmat(centre-this.centre,[N,1]);
            this.centre = centre;
            
        end
        
        function setRotation(this, R)
            % Sets the orientation of the beam, rotating about its centre
            % setRotation(model, R)
            %   R: 3x3 rotation matrix for the beam, where rotation
            %      is applied by post-multiplication: y = xR
            
            N = getNumNodes(this);
            C = repmat(this.centre,[N,1]);
            this.nodes = this.nodes - C;
            this.nodes = this.nodes*((this.rotation')*R);
            this.nodes = this.nodes + C;
            
            this.rotation = R;
        end
        
        
    end
    
    %% Overriden methods
    methods
        
        function addNode(~)
            error('Cannot explicitly add a node to a beam model');
        end
        
        function addElement(~)
            error('Cannot explicitly add an element to a beam model');
        end
        
        function [idx, elem, N] = findContainingElement(this, x)
            % Finds the element that contains point x
            %
            % [idx, elem] = findContainingElement(model, x)
            %   Finds the element that contains the point x, if indeed the
            %   point does fall within the model.  Returns the index of the
            %   element, or 0 if not found.
            %
            %   x:    Kx3 point in spatial coordinates
            %   idx:  element index if found, 0 otherwise
            %   elem: fem_element object
            %   N:    cell array of shape functions evaluated at the point,
            %         such that x can be recovered from the element's node
            %         locations
            
            K = size(x,1);
            Y = repmat(this.widths,[K,1]);
            x =((x-repmat(this.centre,[K,1]))*(this.rotation')+...
                Y/2);
            x = x./Y;
            
            goodIdxs = find( (x(:,1) >= 0) & (x(:,1) <= 1) & ...
                (x(:,2) >= 0) & (x(:,2) <= 1) & ...
                (x(:,3) >= 0) & (x(:,3) <= 1) );
            
            % find nearest element
            x = x.*repmat(this.enum,[K,1]);
            fx = floor(x);
            
            idx = zeros(K,1);
            elem{K} = [];
            N{K} = [];
            
            idx(goodIdxs) = sub2ind(this.enum,fx(goodIdxs,1)+1, ...
                fx(goodIdxs,2)+1, fx(goodIdxs,3)+1);
            elem{goodIdxs} = this.elems{idx(goodIdxs)};
            
            ncoords = 2*(x(goodIdxs,:)-fx(goodIdxs,:))-1;
            N{goodIdxs} = fem_hex_element.getShapeFunction(...
                ncoords(:,1), ncoords(:,2), ncoords(:,3));
            
        end
        
        function [idx, dist] = findNearestNode(this, x)
            % Finds the node nearest to x
            %
            % [idx, dist] = findNearestNode(model, x)
            %   Finds the node nearest to location x
            %
            %   x:    Kx3 input location(s)
            %   idx:  index of resulting nearest node
            %   dist: distance to nearest node
            
            K = size(x,1);
            Y = repmat(this.widths,[K,1]);
            x =((x-repmat(this.centre,[K,1]))*(this.rotation')+...
                Y/2);
            x = x./Y;
            
            goodIdxs = find( (x(:,1) >= 0) & (x(:,1) <= 1) & ...
                (x(:,2) >= 0) & (x(:,2) <= 1) & ...
                (x(:,3) >= 0) & (x(:,3) <= 1) );
            
            % find nearest node using a rounding approach for points inside
            % the model
            x = x.*repmat(this.enum,[K,1]);
            rx = round(x);
            
            idx = zeros(K,1);
            dist = zeros(K,1);
            
            idx(goodIdxs) = sub2ind(this.enum+1,rx(goodIdxs,1)+1, ...
                rx(goodIdxs,2)+1, rx(goodIdxs,3)+1);
            
            D = this.nodes(idx(goodIdxs),:) - x(goodIdxs);
            dist(goodIdxs) = sum(D.*D,2);
            
            % Use old loop-like method for points outside of the model
            badIdxs = setdiff(1:K, goodIdxs);
            if (numel(badIdxs) > 0)
                x = x(badIdxs,:);
                M = size(x,1);
                x = permute(x,[3 2 1]);
                diff = repmat(this.nodes,[1,1,M])-repmat(x,[N,1,1]);
                diff = sum(diff.*diff,2);
                [dist(badIdxs), idx(badIdxs)] = min(diff,[],1);
            end
            
            % convert squared distances to true distances
            dist = sqrt(dist(:));
            
        end
        
        function [ P ] = getInterpolationMatrix( this, X )
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
            
            K = size(X,1);
            Y = repmat(this.widths,[K,1]);
            X =((X-repmat(this.centre,[K,1]))*(this.rotation')+...
                Y/2);
            X = X./Y;
            
            goodIdxs = find( (X(:,1) >= 0) & (X(:,1) <= 1) & ...
                (X(:,2) >= 0) & (X(:,2) <= 1) & ...
                (X(:,3) >= 0) & (X(:,3) <= 1));
            badIdxs = setdiff(1:K, goodIdxs);
            
            % find nearest element
            X = X.*repmat(this.enum,[K,1]);
            fx = floor(X);
            
            nx = this.enum(1)+1;
            ny = this.enum(2)+1;
            nxy = nx*ny;
            
            n1 = fx(:,1)+fx(:,2)*nx+fx(:,3)*nxy+1;
            n2 = n1+1;
            n3 = n1+1+nx;
            n4 = n1+nx;
            n5 = n1+nxy;
            n6 = n5+1;
            n7 = n5+1+nxy;
            n8 = n5+nxy;
            
            c = [n1 n2 n3 n4 n5 n6 n7 n8]';
            c = c(:);
            c( c>getNumNodes(this) ) = 1;
            r = repmat((1:K)',[1 8])';
            r = r(:);
            
            ncoords = 2*(X-fx)-1;
            N = fem_hex_element.getShapeFunction(...
                ncoords(:,1), ncoords(:,2), ncoords(:,3));
            N(badIdxs,:) = 0;
            s = reshape(N',[],1);
             
            P = sparse(r, c, s, K, getNumNodes(this));
        
        end
    end

end

