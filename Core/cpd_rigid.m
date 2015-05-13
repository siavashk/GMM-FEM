function [ TY, R, t, s, P, sigma2 ] = cpd_rigid( X, Y, w, errtol, maxiters, R, t, s, sigma2)
%CPD_RIGID performs the rigid CPD algorithm
%   [ TY, R, t, s, P, sigma2 ] = cpd_rigid( X, Y, w, errtol, maxiters, R, t, s, sigma2)
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
%   R    Initial D x D rotation matrix (optional, defaults to identity)
%   t    Initial D x 1 translation vector (optional, defaults to zeroes)
%   s    Initial scale factor (optional, defaults to 1)
%   sigma2  Initial estimate of squared variance (optional, default
%        estimated from X, Y)
%   
%
%   Outputs:
%   TY   Transformed points
%   (s, R, t)  Final transformation
%   P    Alignment probabilities
%   sigma2  Estimated probability variance

    D = size(X,2);
    N = size(X,1);
    M = size(Y,1);
    scaleFlag = 1; % use scaling
    
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
    
    if (nargin < 6 || isempty(R))
        R = eye(D);
    end
    if (nargin < 7 || isempty(t))
        t = zeros(D, 1);
    end
    if (nargin < 8 || isempty(s))
        s = 1;
    end
    
    if ( s==0 )
        scaleFlag=0;
        s = 1;
    end
    
    % initial transformed input
    TY = bsxfun(@plus, s*Y*(R'), t');
    
    if (nargin < 9 || isempty(sigma2))
        
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
        
        clear diff;
        % estimate initial variance
        % sigma2 = cpd_Sigma2(X,Y);
    end
    
    iters = 0;
    C = ones(D,1);  % temp, so we don't keep recreating
    err = errtol+1;  % initialize so we enter loop
    q = -err-N*D/2*log(sigma2);   % force first q to be > errtol away from next
    
    while ((iters < maxiters) && (err > errtol))
        
        % E-step
        [P, P1, Pt1, Np] = cpd_P(X, TY, sigma2, w);
        
        % M-step
        % Np = sum(P(:));
        mux = sum(P*X,1)/Np;
        muy = sum(P'*Y,1)/Np;
        
        XX = bsxfun(@minus, X, mux);
        YY = bsxfun(@minus, Y, muy);
        % XX = X-repmat(mux,[N,1]);
        % YY = Y-repmat(muy,[M,1]);
        A = XX'*P'*YY;
        
        if ( sum(sum(isnan(P))) > 0 )
            break;
        end
        
        [U, ~, V] = svd(A);
        C(D) = det(U*V');
        R = U*diag(C)*V';
        yPy = (P1')*sum(YY.*YY,2);
        s = trace(A'*R)/yPy;
        if ( ~scaleFlag )
            s=1;
        end        
        t = mux' - s*R*(muy');

        % output
        TY = bsxfun(@plus, s*Y*(R'), t');
        % TY = s*Y*(R')+repmat(t',[M 1]);
        
        qprev = q;
        
        % estimate error (fast)
        xPx = (Pt1')*sum(XX.*XX,2);
        trAR = trace(A'*R);
        % q = cpd_Q(X, TY, P, P1, Pt1, sigma2);
        q = (xPx-2*s*trAR+s*s*yPy)/(2*sigma2) + Np*D/2*log(sigma2);
        err = abs(q-qprev);
        
        % re-estimate variance (fast)
        % sigma2 = cpd_Sigma2(X, TY, P, P1, Pt1);
        sigma2 = (xPx-s*trAR)/(Np*D);
        if (sigma2 <= 0)
            sigma2 = errtol/10; % fix for near zero sigma
        end
        
        iters = iters+1;
    end

end

