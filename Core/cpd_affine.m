function [ TY, B, t, P, sigma2 ] = cpd_affine( X, Y, w, errtol, maxiters, B, t, sigma2)
%CPD_AFFINE performs the affine CPD algorithm
%   [ TY, B, t, P, sigma2 ] = cpd_affine( X, Y, w, errtol, maxiters, B, t, sigma2)
%
%   Inputs:
%   X    N x D matrix of input points
%   Y    M x D matrix of Gaussian centres
%   w    Weight to account for noise and outliers (optional, defaults to 0)
%   errtol Error tolerance for convergence.  Algorithm will terminate if
%        the objective function does not change by more than this amount
%        (optional, defaults to 1e-10)
%   maxiters Maximum number of iterations.  Algorithm will terminate if
%        this many iterations have been performed (optional, defaults to
%        100)
%   B    Initial D x D affine matrix (optional, defaults to identity)
%   t    Initial D x 1 translation vector (optional, defaults to zeroes)
%   sigma2  Initial estimate of squared variance (optional, default
%        estimated from X, Y)
%   
%
%   Outputs:
%   TY   Transformed Gaussian centres
%   (B, t) Resulting affine transform
%   P    Alignment probabilities
%   sigma2 Extimated probability variance

    D = size(X,2);
    N = size(X,1);
    M = size(Y,1);
    
    % allow variable input args
    if (nargin < 3 || isempty(w))
        w = 0;
    end
    if (nargin < 4 || isempty(errtol))
        errtol = 1e-10;
    end
    if (nargin < 5 || isempty(maxiters))
        maxiters = 100;
    end
    
    if (nargin < 6 || isempty(B))
        B = eye(D);
    end
    if (nargin < 7 || isempty(t))
        t = zeros(D, 1);
    end
    
    % initial transformed input
    % TY = Y*(B')+repmat(t',[M 1]);
    TY = bsxfun(@plus, Y*(B'), t');
    
    if (nargin < 8 || isempty(sigma2))
        
        % estimate initial variance
        XX = reshape(X, [1, N, D]);
        YY = reshape(TY, [M, 1, D]);
        XX = repmat(XX, [M, 1, 1]);
        YY = repmat(YY, [1, N, 1]);
        diff = XX-YY;
        diff = diff.*diff;
        err = sum(diff(:));
        sigma2 = 1/(D*N*M)*err;
        clear diff err;
    end
    
    iters = 0;
    err = errtol+1;  % initialize so we enter loop
    q = -err-N*D/2*log(sigma2);   % force first q to be > errtol away from next
    
    while ((iters < maxiters) && (err > errtol))
        
         % E-step
        [P, P1, Pt1, Np] = cpd_P(X, TY, sigma2, w);
        
        % M-step
        mux = sum(P*X,1)/Np;
        muy = sum(P'*Y,1)/Np;
        
        XX = bsxfun(@minus, X, mux);
        YY = bsxfun(@minus, Y, muy);
        % XX = X-repmat(mux,[N,1]);
        % YY = Y-repmat(muy,[M,1]);
        A = XX'*P'*YY;
        
        YPY = (YY'*diag(P1)*YY);
        
        % account for possible singular matrix, checking
        % condition number.  If poorly conditioned,
        % use pseudo-inverse
        if (cond(YPY) < 1e10)
            B = A/YPY;
        else
            B = A*pinv(YPY);
        end
        t = mux' - B*(muy');
        
        % output
        % TY = Y*(B')+repmat(t',[M 1]);
        TY = bsxfun(@plus, Y*(B'), t');
               
        qprev = q;
        
        % estimate error (fast)
        trAB = trace(A*B');
        xPx = (Pt1')*sum(XX.*XX,2);
        trBYPYB = trace(B*YPY*B');
        % q = cpd_Q(X, TY, P, P1, Pt1, sigma2);
        q = (xPx-2*trAB+trBYPYB)/(2*sigma2)+D*Np/2*log(sigma2);
        err = abs(q-qprev);
        
        % Update variance (fast)
        % sigma2 = cpd_Sigma2(X, TY, P, P1, Pt1);
        sigma2 = (xPx-trAB)/(Np*D);
        if (sigma2 <= 0)
            sigma2 = errtol/10; % fix for near zero sigma
        end
        
        iters = iters+1;
    end
end

