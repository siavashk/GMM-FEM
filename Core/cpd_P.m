function [ P, P1, Pt1, Np ] = cpd_P( X, TY, sigma2, w)
%CPD_P Computes the probability matrix for the CPD algorithm
%
%   P = cpd_P(x, Ty, sigma2, w) computes the CPD probability matrix 
%
%   Inputs:
%   X    N x D matrix of the data points
%   TY   M x D matrix of transformed Gaussian centres
%   sigma2  Standard deviation of Gaussians
%   w    weight to account for noise and outliers (optional, defaults to 0)
%   Outputs:
%   P    M x N matrix of assignment
%   P1   M x 1 vector, sum(Pij,j=1..N)
%   Pt1  N x 1 vector, sum(Pij',i=1..M);
%   Np   Summation over all elements of P

    % default to w = 0
    if (nargin < 4 || isempty(w)) 
        w = 0;
    end
    
    N = size(X,1);
    M = size(TY,1);
    D = size(X,2);
    
     % Reshape and order for easier matrix computation
    P = zeros(M, N);
    for i=1:M
        P(i,:) = P(i,:) + sum( bsxfun(@minus, X, TY(i,:)).^2, 2 )';
    end
    
    c = (2*pi*sigma2)^(D/2)*w/(1-w)*M/N;

    P = exp( -P/(2*sigma2) );
    den = sum(P, 1)+c;
    den = repmat(den, [M, 1] );
    P = P./den;
    
    Pt1 = sum(P,1)';
    P1 = sum(P,2);
    Np = sum(P1);
    
    %{
    % compare with summation
    Ptest = zeros(M,N);
    for n=1:N
        msum = 0;
        for m=1:M   
            vtmp =  exp(-normsq(X(n,:)-TY(m,:),2)/(2*sigma2));
            Ptest(m,n) = vtmp;
            msum = msum + vtmp;
        end
        msum = msum + c;

        for m=1:M
            Ptest(m,n) = Ptest(m,n)/msum;
        end
    end
    if (max(abs(P(:)-Ptest(:))) > 1e-13)
        fprintf('something is wrong\n');
    end
    %}
    
end

