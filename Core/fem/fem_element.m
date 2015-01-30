classdef fem_element < handle
    %FEM_ELEMENT Abstract class for a 3D finite element
    %   Extended by fem_tet_element and fem_hex_element
    %
    %   Copyright 2013 C. Antonio Sanchez [antonios@ece.ubc.ca]
    
    %% Properties
    properties
        % Indices of nodes belonging to this element
        % Node ordering is expected to be CW w.r.t. the outward normal
        nodeIdxs = [];
    end
    
    %% Abstract methods
    methods %(Abstract)  // Abstract not properly supported on GNU Octave
        
        function N = getNumNodes(this)
        % Returns number of nodes belonging to the element
            N = numel(nodeIdxs);
        end
        
        function N = getN(this,xi,eta,mu)
        % Returns the shape function evaluated at natural coords
        %
        %   N = getN(elem, xi, eta, mu)
        %       Returns the 1xN vector of shape functions evaluated at the
        %       natural coordinates [xi, eta, mu].  N is the number of
        %       nodes belonging to this element.
            N = [];
        end
        
        function dNds = getdN(this,xi,eta,mu)
        % Returns the shape function gradient in natural coords
        %
        % dNds = getdN(elem, xi, eta, mu)
        %      Returns the 3xN shape function gradient in/at natural
        %      coordinates [xi, eta, mu].  The ith column of dN
        %      corresponds to the ith shape function gradient.
        %      This often needs to be transformed into spatial
        %      coordinates.
            dNds = [];
        end
        
        function out = isNaturalInside(this, xem, tol)
        % Determins whether natural coordinates fall inside the element
        %
        % [out] = isNaturalInside(elem, xem, tol)
        %   Returns 1 if supplied natural coordinates [xi, eta, mu]
        %   lie inside the boundaries of this element, with supplied
        %   tolerance
            out = [];
        end
        
        function [ipnts, w] = getIPnts(this, order)
        % Returns the cubature points and associated weights
        %
        %  [ipnts, w] = getIPnts(elem, order)
        %     Returns the cubature points for performing numerical
        %     integrations over the element.
        %
        %     order: Numerical order of integration (default determined by
        %            element type)
        %     ipnts: Mx3 cubature points in natural coordinates
        %     w:     1xM vector of cubature weights
            ipnts = [];
            w = [];
        end
        
    end
    
    %% Static methods
    methods(Static)
        function elems = create(nodeIdxs)
            % Creates a set of appropriate finite elements
            %
            %  elems = create(nodeIdxs)
            %       nodeIdxs:   MxD matrix of node indices, where M is the
            %                   number of elements to create, and D is the
            %                   number of nodes per element.
            
            switch(size(nodeIdxs,2))
                case 4
                    elems = fem_tet_element.create(nodeIdxs);
                case 5
                    elems = fem_pyramid_element.create(nodeIdxs);
                case 6
                    elems = fem_wedge_element.create(nodeIdxs);
                case 8
                    elems = fem_hex_element.create(nodeIdxs);
                otherwise
                    warning('Unknown number of nodes.  Element ignored.');
            end
        end
    end
    
    %% Protected static methods
    methods (Access=protected)
        
        function [pnts, w] = getGaussQuadrature( order )
            % Returns a 1D Gaussian quadrature rule for a given order
            switch (order)
                case 0
                    pnts = 0;
                    w = 2;
                case 1
                    pnts = [-1/sqrt(3); 1/sqrt(3)];
                    w = [1; 1];
                case 2
                    b=sqrt(3/5);
                    pnts=[-b; 0; b];
                    w=[5/9; 8/9; 5/9];
                case 3
                    b=1/35*(sqrt(525-70*sqrt(30)));
                    b2=1/35*(sqrt(525+70*sqrt(30)));
                    pnts = [b; -b; b2; -b2];
                    w = [ (18+sqrt(30))/36*[1;1]; (18-sqrt(30))/36*[1;1]];
                case 4
                    b=1/21*sqrt(245-14*sqrt(70));
                    b2=1/21*sqrt(245+14*sqrt(70));
                    pnts=[0; b; -b; b2; -b2];
                w = [128/225;
                    (322+13*sqrt(70))/900*[1;1];
                    (322-13*sqrt(70))/900*[1;1]];
            end
        end
        
        
    end
    
    %% Access methods
    methods
        function nodeIdxs = getNodeIdxs(this)
            % Returns the node indices associated with this element
            nodeIdxs = this.nodeIdxs;
        end
        
        function J = getJ(~,dNds,X)
            %  Computes the spatial->natural Jacobian matrix
            %
            %  J = getJ(elem, dNds, X)
            %      Computes the coordinate Jacobian matrix
            %      J = d(x,y,z)/d(xi,eta,mu) evaluated at natural coordinates.
            %
            %      dNds: the shape gradient in natural coordinates
            %      X:  Nx3 spatial coordinates of nodes, where N is the number
            %          of nodes belonging only to this element
            J = dNds*X;
        end
        
        function dNdx = getShapeGrad(~,dNds,J)
            %  Computes the shape function gradient in spatial coords
            %
            %  dNdx = getShapeGrad(elem, dNds, J)
            %      Computes and returns the 3xN shape function in spatial
            %      coordinates.
            %
            %      dNds: the shape gradient in natural coordinates
            %      J: the spatial->natural Jacobian matrix. See getJ(...)
            dNdx = J\dNds;
        end
        
        function [ x ] = getSpatialCoords(this, X, xi, eta, mu)
            % Computes the spatial coordinates of a point inside this element
            %
            % x = getSpatialCoords(elem, X, xi, eta, mu)
            %   Computes the 1x3 spatial coordinates of a point inside the 
            %   supplied element
            %
            %   X:  the Nx3 locations of the nodes belonging to this element
            %   [xi, eta, mu]: the natural coordinates inside the element
            
            N = getN(this, xi, eta, mu);
            x = (N*X);
            
        end
        
        function [ xem ] = getNaturalCoords(this, X, x, xem, maxiters, tol)
            % Computes the natural coordinates of point x in this element
            %
            % xem = getNaturalCoords(elem, X, x, xem, maxiters, tol)
            %   Uses a modified Newton's method to compute the natural
            %   coordinates of a point w.r.t. the element nodes.
            %
            %   X:  the Nx3 locations of the nodes belonging to this element
            %   x:  the 1x3 spatial coordinates of the point for which to
            %       determine coordinates
            %   xem: the 1x3 initial guess  (optional, default=[0;0;0])
            %   maxiters: maximum number of iterations (optional, default=20)
            %   tol: error tolerance before terminating (optional,
            %        default=1e-12)
            
            if (nargin < 4 || isempty(xem))
                xem = [0 0 0];
            end
            if (nargin < 5 || isempty(maxiters))
                maxiters = 20;
            end
            if (nargin < 6 || isempty(tol))
                tol = 1e-12;
            end
            
            for i=1:maxiters
                dNds = getdN(this, xem(1), xem(2), xem(3));
                N = getN(this, xem(1), xem(2), xem(3));
                J = getJ(this, dNds, X);
          
                dxds = zeros(3,3);
                for k=1:numel(N)
                    dxds = dxds + dNds(:, k)*X(k,:);
                end
                
                % residual
                pos = (N*X);
                res = pos-x;
                
                % update
                del = res/(J);
                if (norm(del) < tol)
                    break;
                end
                
                xem = xem - del;
                
            end
            
        end
        
        function [ xem ] = getNaturalCoordsRobust(this, X, x, xem, maxiters, tol)
            % Computes the natural coordinates of point x in this element
            %
            % xem = getNaturalCoords(elem, X, x, xem, maxiters, tol)
            %   Uses a modified Newton's method to compute the natural
            %   coordinates of a point w.r.t. the element nodes.
            %
            %   X:  the Nx3 locations of the nodes belonging to this element
            %   x:  the 1x3 spatial coordinates of the point for which to
            %       determine coordinates
            %   xem: the 1x3 initial guess  (optional, default=[0;0;0])
            %   maxiters: maximum number of iterations (optional, default=20)
            %   tol: error tolerance before terminating (optional,
            %        default=1e-12)
            
            if (nargin < 4 || isempty(xem))
                xem = [0 0 0];
            end
            if (nargin < 5 || isempty(maxiters))
                maxiters = 20;
            end
            if (nargin < 6 || isempty(tol))
                tol = 1e-12;
            end
            
            for i=1:maxiters
                dNds = getdN(this, xem(1), xem(2), xem(3));
                N = getN(this, xem(1), xem(2), xem(3));
                J = getJ(this, dNds, X);
                
                % residual
                pos = (N*X);
                res = pos-x;
                
                % update
                del = res/(J);
                if (norm(del) < tol)
                    xem = xem - del;
                    break;
                end
                
                % don't jump too far, use binary search
                r2 = (res*res');
                nr2 = 2*r2; % initialize so we run through loop once
                oldxem = xem;
                while (nr2 > r2 && norm(del) >= tol)
                    xem = oldxem - del;
                    N = getN(this, xem(1), xem(2), xem(3));
                    nres = (N*X)-x;
                    nr2 = (nres*nres');
                    del = del/2;
                end
                
            end
            
        end
        
        function [in, xem] = isInside(this, X, x, xem, maxiters, tol)
            % Determines whether a point is inside this element
            %
            % [in, xem] = isInside(elem, X, x)
            %   Determines whether the point x in spatial coordinates falls
            %   inside an element with node locations given by X.  This method
            %   computes the natural coordinates of the point in this element,
            %   then determines whether those natural coordinates fall within
            %   appropriate bounds.
            %
            %   Inputs:
            %   X:  the Nx3 locations of the nodes belonging to this element
            %   x:  the 1x3 spatial coordinates of the point for which to
            %       determine coordinates
            %   xem: the 1x3 initial guess  (optional, default=[0;0;0])
            %   maxiters: maximum number of iterations (optional, default=20)
            %   tol: error tolerance before terminating (optional,
            %        default=1e-12)
            %
            %   Outputs:
            %   in:  1 if point is inside
            %   xem: natural coordinates of point x in this element
            
            if (nargin < 4 || isempty(xem))
                xem = [0, 0, 0];
            end
            if (nargin < 5 || isempty(maxiters))
                maxiters = 20;
            end
            if (nargin < 6 || isempty(tol))
                tol = 1e-12;
            end
            
            xem = getNaturalCoords(this, X, x, xem, maxiters, tol);
            in = isNaturalInside(this, xem, tol);
            
        end
        
        function B = getB(this, dNdx)
            % Returns the full B matrix required for stiffness computations
            %
            %  B = getB(elem, dNdx)
            %      Returns the 6x3N extended B matrix required for
            %      stiffness computations.  The integrand for the stiffness
            %      matrix is given by B'DB, where D is the 6x6 elasticity
            %      matrix of the material.  The columns [3i-2, 3i-1, 3i]
            %      correspond to Bi, the ith B matrix.
            %
            %      dNdx: the 3xN shape function gradient in spatial
            %            coordinates.  See getShapeGrad(...)
            
            % slow loop, may want to override in element classes
            N = getNumNodes(this);
            B = zeros([6,3*N]);
            for i=1:N
                B(6,(3*i-2):3*i) = [diag(dNdx(:,i));
                    dNdx(2,i), dNdx(1,i), 0;
                    dNdx(3,i), 0, dNdx(1,i);
                    0, dNdx(3,i), dNdx(2,i)];
            end
            
        end
        
        function [K, minJ] = getLinearK(this, D, X)
            % Computes the linear stiffness matrix for this element
            %
            % [K, minJ] = getLinearK(elem, D, X)
            %   Computes the element stiffness matrix, and optionally
            %   returns the minimum deformation gradient determinant,
            %   useful for determining if an element has become inverted
            %
            %   D:  the 6x6 elasticity matrix
            %   X:  the Nx3 matrix respresenting the set of nodes belonging 
            %       to this element
            
            [ipnts, w] = getIPnts(this);
            n = getNumNodes(this);
            K = zeros([3*n,3*n]);
            minJ = inf;
                
            % linear material
            for i=1:length(w)
                dNds = getdN(this,ipnts(i,1),ipnts(i,2),ipnts(i,3));
                J = getJ(this,dNds,X);
                detJ = det(J);
                if (detJ < minJ)
                    minJ = detJ;
                end
                dNdx = getShapeGrad(this,dNds, J);
                B = getB(this,dNdx);
                K = K + w(i)*detJ*(B'*D*B);
            end
            
        end
        
        function M = getM(this, X)
            % Computes the mass matrix for this element
            %
            % M = getM(elem, X)
            %   Computes the mass matrix, assuming constant density (=1)
            %
            %   X:  the Nx3 matrix respresenting the set of nodes belonging to
            %       this element
            
            [ipnts, w] = getIPnts(this);
            n = getNumNodes(this);
            M = zeros([n,n]);
            for i=1:length(w)
                dNds = getdN(this,ipnts(i,1),ipnts(i,2),ipnts(i,3));
                N = getN(this, ipnts(i,1), ipnts(i,2), ipnts(i,3));
                J = getJ(this,dNds,X);
                
                M = M + w(i)*det(J)*(N'*N);
            end
            
            % expand to 3n x 3n
            M = kron(M, eye(3));
            
        end
        
        function [K, minJ] = getStiffnessMatrix(this, D, nodeArray)
            % Returns the stiffness matrix for the element
            %
            % [K, minJ] = getStiffnessMatrix(elem, D, nodeArray)
            %   Computes the element stiffness matrix.  Optionally returns
            %   the minimum computed deformation gradient determinant,
            %   useful for determining if an element is inverted.
            %
            %   D:  the 6x6 elasticity matrix
            %      OR
            %      Object of type fem_material, which will compute a 6x6
            %      elasticity matrix with 
            %          getStiffnessMatrix(D, this, nodeArray)
            %   nodeArray: the Nx3 matrix respresenting the set of ALL
            %       nodes, not just the ones belonging to this element
            
            if (isa(D,'fem_material'))
                [K, minJ] = getStiffnessMatrix(D, this, nodeArray);
            else
                [K, minJ] = getLinearK(this, D, nodeArray(this.nodeIdxs,:));
            end
        end
        
        function M = getMassMatrix(this, nodeArray)
            % Returns the mass matrix for the element
            %
            % M = getMassMatrix(elem, D, nodeArray)
            %   Computes the element mass matrix
            %
            %   nodeArray: the Nx3 matrix respresenting the set of ALL
            %       nodes, not just the ones belonging to this element
            M = getM(this, nodeArray(this.nodeIdxs,:));
        end
        
        function delete(this)
            % nothing
        end
        
    end
    
end

