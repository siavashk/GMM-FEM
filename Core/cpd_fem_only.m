function [TY, R, t, s, V, sigma2, P, fem, U, Phi] = cpd_fem_only(X, Y, F, w, errtol, maxiters, R, t, s, sigma2, beta, E, nu, ...
    fem, Phi, FV)
%cpd_biomech performs biomechanically constrained point cloud registration
%   [TY, sigma2] = cpd_biomech(X, Y, w, errtol, maxiters, R, t, s, sigma2, beta )
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
%   R    Initial D x D affine matrix (optional, defaults to identity)
%   t    Initial D x 1 translation vector (optional, defaults to zeroes)
%   s    single scale value
%   beta    Weight to incorporate FEM
%   sigma2  Initial estimate of squared variance (optional, default
%        estimated from X, Y)
%
%   Outputs:
%   TY   Transformed points
%   (B, t)  Final transformation
%   P    Alignment probabilities
%   sigma2  Estimated probability variance

EPSILON = 1e-16;	% default small value

D = size(X,2);
N = size(X,1);
M = size(Y,1);

% allow variable input args
if (nargin < 4 || isempty(w))
    w = 0;
end
if (nargin < 5 || isempty(errtol))
    errtol = 1e-5;
end
if (nargin < 6 || isempty(maxiters))
    maxiters = 100;
end
if (nargin < 7 || isempty(R))
    R = eye(D);
end
if (nargin < 8 || isempty(t))
    t = zeros(D, 1);
end
if (nargin < 9 || isempty(s))
    s = 1;
end

% Initialize the registration
y_vec = reshape(Y',[],1);
v_vec = zeros(D*M,1);

Y = reshape(y_vec,D,[])';
TY = reshape(y_vec+v_vec,D,[])';
TY = bsxfun(@plus, TY*(R')*s, t');

if (nargin < 10 || isempty(sigma2))
    % estimate initial variance
    XX = reshape(X, [1, N, D]);
    YY = reshape(TY, [M, 1, D]);
    XX = repmat(XX, [M, 1, 1]);
    YY = repmat(YY, [1, N, 1]);
    diff = XX-YY;
    diff = diff.*diff;
    err2 = sum(diff(:));
    sigma2 = 1/(D*N*M)*err2;
    clear diff;
end

if (nargin < 14 || isempty(fem))
    [nodes, elems] = tetgen(Y', F', 2);
    
    fem = fem_model(nodes', elems');
    setAbortJacobian(fem, EPSILON);      % abort on inverted elements
end

if (nargin < 15 || isempty(Phi))
    
    NN = size(nodes,2);
    diff = (nodes(:,1:M)-Y');
    if (norm(diff(:)) > EPSILON)
        % surface nodes were added, need to adjust Phi accordingly
        cols = zeros(1,M);
        nidx = 1;
        for i=1:size(Y,1)
            nerr = nodes(:,nidx) - Y(i,:)';
            % search forward for correct node id
            while ( (norm(nerr) > EPSILON) )
                nidx = nidx+1;
                nerr = nodes(:,nidx) - Y(i,:)';
            end
            cols(i) = nidx;
            nidx = nidx+1;
        end
        Phi = sparse(1:M, cols, ones(1,M), M, NN);
    else
        Phi = sparse(1:M, 1:M, ones(1,M), M, NN);
    end
    
    clear diff cols nidx nerr NN;
    
end

if (nargin < 16 || isempty(FV))
    FV = [];
end

% FEM stuff
D_material = fem_material_linear.getElasticity(E, nu); 
% compute stiffness
[K, minJ] = getStiffnessMatrix(fem, D_material);
if (minJ < 0) 
    error('Stiffness matrix has inverted elements');
end
Phi_tilde = kron(Phi, eye(D));

iters = 0;
err = errtol+1;

% C = ones(D,1);  % temp, so we don't keep recreating, for rigid transform    
u_vec = [];
while ((iters < maxiters) && (err > errtol))
    
    % E-step
    [P, P1, Pt1, Np] = cpd_P(X, TY, sigma2, w);
    P1(P1<1e-10) = 1e-10;
        
    % Here, TY is just Y+V
    % TY = y_vec+v_vec;
    % TY = reshape(TY,D,[])';
    
    % M-step
    % Rigid align
    %     mux = sum(P*X,1)/Np;
    %     muy = sum(P'*TY,1)/Np;
    % 
    %     % recompute because P changed, causing mux/y to change
    %     XX = bsxfun(@minus, X, mux);
    %     YY = bsxfun(@minus, TY, muy);
    %     A = XX'*P'*YY;    
    %     [U, ~, V] = svd(A);
    %     C(D) = det(U*V');
    %     R = U*diag(C)*V';
    %     s = trace(A'*R)/trace(YY'*diag(P1)*YY);
    %     t = mux' - s*R*(muy');

    % FEM step
    dP1 = spdiags(kron(P1,ones(D,1)),0,D*M,D*M);
    LHS = s*s*(Phi_tilde')*dP1*Phi_tilde + beta*sigma2*K;
    RHS = -s*(P*X)*R;
    RHS = -Phi_tilde'*(reshape(RHS',[],1)+dP1*(s*s*y_vec+s*repmat(R'*t,M,1)));
    s1 = warning('error', 'MATLAB:singularMatrix'); %#ok<CTPCT>
    warning('error', 'MATLAB:singularMatrix'); %#ok<CTPCT>
    if ( ~isempty(u_vec) )
        u_vec_old = u_vec;
    end
    try
      % Regular processing part
      u_vec = LHS\RHS;
    catch err1
      % Exception-handling part
      fprintf('Can''t solve linear system (reason: %s)\n', err1);
      u_vec = u_vec_old;
      iters = maxiters;
    end
    warning(s1);
    v_vec = Phi_tilde*u_vec;
    
    % Update GMM centroids
    TY = y_vec+v_vec;
    TY = reshape(TY,D,[])';
    TY = bsxfun(@plus, TY*(R')*s, t');
    
    % Update sigma
    PX = P*X;
    xPx = (Pt1')*sum(X.*X,2);
    yPy = (P1')*sum(TY.*TY,2);
    trPXTY = sum(TY(:).*PX(:));
    
    err = sigma2;
    sigma2 = (xPx-2*trPXTY+yPy)/(Np*D);
    err = abs((err-sigma2)/err);	% percent change
    disp(['Error fraction:', num2str(err)]);
    
        [az, el] = view;
        clf;
        if (isempty(FV))
            plot3(X(:,1), X(:,2), X(:,3),'.r','MarkerSize',10);
        else 
            patch(FV,'FaceColor','red','FaceAlpha',0.2, 'EdgeColor', 'red', 'EdgeAlpha',0.2);
        end
        hold on;
        % axis([-3,3,-3,3,-3,3]);
        % without deformation
        TYstiff = y_vec;
        TYstiff = reshape(TYstiff,D,[])';
        TYstiff = bsxfun(@plus, TYstiff*(R')*s, t');
        YFV.vertices = TY;
        YFV.faces = F;
        plot3(TYstiff(:,1), TYstiff(:,2), TYstiff(:,3),'og','MarkerSize', 5, 'MarkerFaceColor', 'g');
        
        patch(YFV,'FaceColor','blue','FaceAlpha',0.2, 'EdgeColor', 'blue', 'EdgeAlpha',0.2);
        % plot3(TY(:,1), TY(:,2), TY(:,3),'ob','MarkerSize', 5, 'MarkerFaceColor', 'b');
        % axis([-3,3,-3,3,-3,3]);
        legend({'target','rest','fem'},'location','NEo')
        view(az, el);
        drawnow;
        
    iters = iters + 1;
end

V = reshape(v_vec,D,[])';
U = reshape(u_vec,D,[])';

end