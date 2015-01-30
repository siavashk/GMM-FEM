function [ TY, P, sigma2 ] = cpd_coherent( X, Y, lambda, beta2, w, errtol, maxiters, sigma2)
%CPD_COHERENT performs the coherent CPD algorithm
%   [ TY, P, sigma2 ] = cpd_coherent( X, Y, lambda, beta, w, errtol, maxiters, sigma2)
%
%   Inputs:
%   X    N x D matrix of input points
%   Y    M x D matrix of Gaussian centres
%   lambda > 0, regularization weight
%   beta2  > 0, factor controlling amount of coherence
%   w    Weight to account for noise and outliers (optional, defaults to 0)
%   errtol Error tolerance for convergence.  Algorithm will terminate if
%        the objective function does not change by more than this amount
%        (optional, defaults to 1e-10)
%   maxiters Maximum number of iterations.  Algorithm will terminate if
%        this many iterations have been performed (optional, defaults to
%        100)
%   sigma2  Initial estimate of squared variance (optional, default
%        estimated from X, Y)
%   
%   Ouputs:
%   TY   Transformed Gaussian points.  The transform can then be recovered
%        by subtracting this from the original points
%   P    Alignment probabilities
%   sigma2 Estimated probability variance

    D = size(X,2);
    N = size(X,1);
    M = size(Y,1);
    
    % allow variable input args
    if (nargin < 5 || isempty(w))
        w = 0;
    end
    if (nargin < 6 || isempty(errtol))
        errtol = 1e-10;
    end
    if (nargin < 7 || isempty(maxiters))
        maxiters = 100;
    end
    
    % initial transformed input
    TY = Y;
    
    if (nargin < 8 || isempty(sigma2))
        
        % estimate initial variance
        % Restored to remove additional function call
        XX = reshape(X, [1, N, D]);
        YY = reshape(TY, [M, 1, D]);
        XX = repmat(XX, [M, 1, 1]);
        YY = repmat(YY, [1, N, 1]);
        diff = XX-YY;
        diff = diff.*diff;
        err2 = sum(diff(:));
        sigma2 = 1/(D*N*M)*err2;
        
        % estimate initial variance
        % sigma2 = cpd_Sigma2(X,Y);
    end
    
    % G matrix
    XX = reshape(Y, [1, M, D]);
    YY = reshape(Y, [M, 1, D]);
    XX = repmat(XX, [M, 1, 1]);
    YY = repmat(YY, [1, M, 1]);
    diff = XX-YY;
    diff = sum(diff.*diff,3);
    G = exp(-diff/(2*beta2));
    clear diff XX YY;
    
    % Transform parameters
    % W = zeros(M,D);
    
    % initialize loop
    iters = 0;
    err = errtol+1;  % initialize so we enter loop
    % q = -Inf;   % force first q to be > errtol away from next
    W = [];
    while ((iters < maxiters) && (err > errtol))
        
        % E-step
        [P, P1, Pt1, Np] = cpd_P(X, TY, sigma2, w);
        
        % M-step
        P1(P1<1e-10) = 1e-10;
        P1i = 1./P1;
        PX = P*X;
        
        % Note: if W doesn't change much, might be better 
        %       to use GMRES, supplying an initial guess
        try
            % Regular processing part
            Wold = W;
            W = (G+lambda*sigma2*diag(P1i))\(diag(P1i)*PX-Y);
        catch
            % Exception-handling part
            fprintf('Can''t solve linear system (reason: %s)\n', lasterr);
            % W = (G+lambda*sigma2*diag(P1i)+(1e-10)*speye(3*M, 3*M))\(diag(P1i)*PX-Y);
            W = Wold;
            iters = maxiters;
        end
        
        
        % output
        TY = Y+G*W;
        % disp(TY(1,:));
        
        % use sigma2 as operator
        qprev = sigma2;
        
        % estimate error 
        xPx = (Pt1')*sum(X.*X,2);
        yPy = (P1')*sum(TY.*TY,2);
        trPXTY = sum(TY(:).*PX(:));
        
        % q = cpd_Q(X, TY, P, P1, Pt1, sigma2);
        % err = abs(q-qprev);
        
        % re-estimate variance (fast)
        % sigma2 = cpd_Sigma2(X, TY, P, P1, Pt1);
        sigma2 = (xPx-2*trPXTY+yPy)/(Np*D);
        if (sigma2 <= 0)
            sigma2 = errtol/10; % fix for near zero sigma
        end
        
        err = abs(sigma2-qprev);
        
        iters = iters+1;
    end

end

