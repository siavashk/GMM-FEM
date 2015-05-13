classdef fem_wedge_element < fem_element
    %FEM_WEDGE_ELEMENT Linear wedge element
    %
    % Node indices are assumed to be ordered as follows:
    %   1:3, nodes of one triangular face ordered clockwise 
    %        w.r.t. outward normal
    %   4:6, corresponding nodes on opposite face (will be CCW)
    % 
    %   Copyright 2013 C. Antonio Sanchez [antonios@ece.ubc.ca]
    
    %% Constants
    properties (Constant, Access=private)
        
        INTEGRATION_COORDS_GAUSS_6 = ...
           [ 1/6, 1/6, -1/sqrt(3), 1/6;
             4/6, 1/6, -1/sqrt(3), 1/6;
             1/6, 4/6, -1/sqrt(3), 1/6;
             1/6, 1/6,  1/sqrt(3), 1/6;
             4/6, 1/6,  1/sqrt(3), 1/6;
             1/6, 4/6,  1/sqrt(3), 1/6];
        
        % number of nodes in wedge elements
        WEDGE_NUM_NODES = 6;
    end
    
    properties(Access=private)
    
        % uninitialized ipnts info
        ipntLength = 0;     % number of cubature points
        % cubature weights
        ipntW = [];  
        % cubature point locations
        ipntLoc = []';    
    end
    
    %% Implemented methods
    methods
        function edgeIdxs = getEdgeIdxs(this)
            nodeIdxs = getNodeIdxs(this);
            edgeIdxs = nodeIdxs([1 2 3 4 5 6 1 2 3;
                                 2 3 1 5 6 4 4 5 6]);
        end
        
        function xem = getEdgeNaturalCoords(~, edgeIdx, t)
            xem = zeros(numel(edgeIdx), 3);
            
            % loop faster than vectorized for small number of inputs
            for i=1:numel(edgeIdx)
                switch (edgeIdx(i))
                    case 1
                        xem(i,:) = [t(i), 0, -1];
                    case 2
                        xem(i,:) = [1-t(i), t(i), -1];
                    case 3
                        xem(i,:) = [0, 1-t(i), -1];
                    case 4
                        xem(i,:) = [t(i), 0, 1];
                    case 5
                        xem(i,:) = [1-t(i), t(i), 1];
                    case 6
                        xem(i,:) = [0, 1-t(i), 1];
                    case 7
                        xem(i,:) = [0, 0, 2*t(i)-1];
                    case 8
                        xem(i,:) = [1, 0, 2*t(i)-1];
                    case 9
                        xem(i,:) = [0, 1, 2*t(i)-1];
                end
            end
            
        end
        
        function n = getNumNodes(~)
            n = fem_wedge_element.WEDGE_NUM_NODES;
        end
        
        function N = getN(~,xi,eta,mu)
            mm = (1-mu);
            mp = (1+mu);
            xes = (1-xi-eta);
            
            N = [xes.*mm, xi.*mm, eta.*mm, xes.*mp, xi.*mp, eta.*mp]/2;
        end
        
        function dN = getdNds(~,xi,eta,mu)
            mm = (1-mu);
            mp = (1+mu);
            xes = (1-xi-eta);
            
            dN = [ -mm,  mm,    0, -mp,  mp,   0;
                   -mm,   0,   mm, -mp,  0,   mp;
                  -xes, -xi, -eta, xes,  xi, eta]/2;
        end
        
        function [in] = isNaturalInside(~, xem)
           s = xem(1)+xem(2);
           in = (xem(1) >= 0 && xem(1) <= 1 && xem(2) >= 0 && ...
                 xem(2) <= 1 && xem(3) >= -1 && xem(3) <= 1 && ...
                 s >= 0 && s <= 1 );
        end
        
        function [ipnts, w] = getIPnts(this)
            ipnts = this.ipntLoc;
            w = this.ipntW;
        end
        
    end
    
    %% Over-ridden methods
    methods
         function B = getB(~, dNdx)
            B = [dNdx(1,1) 0 0  dNdx(1,2) 0 0  dNdx(1,3) 0 0 ...
                    dNdx(1,4) 0 0  dNdx(1,5) 0 0  dNdx(1,6) 0 0;
                 0 dNdx(2,1) 0  0 dNdx(2,2) 0  0 dNdx(2,3) 0 ...
                    0 dNdx(2,4) 0  0 dNdx(2,5) 0  0 dNdx(2,6) 0;
                 0 0 dNdx(3,1)  0 0 dNdx(3,2)  0 0 dNdx(3,3) ...
                    0 0 dNdx(3,4)  0 0 dNdx(3,5)  0 0 dNdx(3,6);
                 dNdx(2,1) dNdx(1,1) 0  dNdx(2,2) dNdx(1,2) 0 ...
                    dNdx(2,3) dNdx(1,3) 0  dNdx(2,4) dNdx(1,4) 0 ...
                    dNdx(2,5) dNdx(1,5) 0  dNdx(2,6) dNdx(1,6) 0;
                 dNdx(3,1) 0 dNdx(1,1)  dNdx(3,2) 0 dNdx(1,2) ...
                    dNdx(3,3) 0 dNdx(1,3)  dNdx(3,4) 0 dNdx(1,4) ...
                    dNdx(3,5) 0 dNdx(1,5)  dNdx(3,6) 0 dNdx(1,6);
                 0 dNdx(3,1) dNdx(2,1)  0 dNdx(3,2) dNdx(2,2) ...
                    0 dNdx(3,3) dNdx(2,3)  0 dNdx(3,4) dNdx(2,4) ...
                    0 dNdx(3,5) dNdx(2,5)  0 dNdx(3,6) dNdx(2,6)];
        end
    end
    
    % private methods
    methods (Access=private)
        % initialization
        function init(this)
            setDefaultIntegration(this);
        end
    end
    
    methods (Static)
        function wedges = create(E)
        % Factory, creates an array of wedge elements
        %
        % wedges = create(E)
        %   Creates a set of M wedge elements given an Mx6 matrix of
        %   node indices, arranged CW w.r.t. the outward normal
        
            % pre-allocate array
            wedges(size(E,1)) = fem_wedge_element(E(end,:));
            for i=1:(size(E,1)-1)
                wedges(i) = fem_wedge_element(E(i,:));
            end
        end
    end
    
    %% Wedge-specific methods
    methods
        function wedge = fem_wedge_element(E)
        % Constructor, creates a single wedge element
        %
        % wedge = fem_wedge_element(E)
        %   Creates a wedge element given a 1x6 vector of
        %   node indices, arranged CW w.r.t. the outward normal
        
            init(wedge);
            if (nargin == 0) 
                wedge.nodeIdxs = [];
            else
                if (size(E,1)<2)
                    wedge.nodeIdxs = E;
                else
                   error('Matlab:fem_wedge_element:invalidinput', ...
                   'Can only create one element at at time');
                end
            end
        end
        
        function setDefaultIntegration(this)
        % Sets default integration to 8-point Gaussian quadrature
            this.ipntLength = 8;
            this.ipntW = fem_wedge_element.INTEGRATION_COORDS_GAUSS_6(:,4)';  
            this.ipntLoc = fem_wedge_element.INTEGRATION_COORDS_GAUSS_6(:,1:3);    
        end
        
    end
end

