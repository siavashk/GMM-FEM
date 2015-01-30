function [ Q ] = cpd_Q( X, TY, P, P1, Pt1, sigma2)
%CPD_Q Computes the objective function for the CPD algorithm
%
%   Q = cpd_Q(x, Ty, Pm sigma2) computes the CPD objective function 
%
%   Inputs:
%   X    N x D matrix of the data points
%   TY   M x D matrix of transformed Gaussian centres
%   P    M x N Probability matrix, computed through cpd_P
%   P1   M x 1 vector, P*ones(N,1)
%   Pt1  N x 1 vector, P'*ones(M,1)
%   sigma2  variance of Gaussians

    D = size(X,2);
    
    Np = sum(P(:));
    
    Q = trace(X'*diag(Pt1)*X -2*X'*P'*TY+...
        TY'*diag(P1)*TY)/(2*sigma2) + Np*D/2*log(sigma2);
    
    %{
    % compare with summation
    Qtest = 0;
    for m=1:size(TY,1)
        for n=1:size(X,1)
            Qtest = Qtest + P(m,n)*normsq(X(n,:)-TY(m,:),2);
        end
    end
    Qtest = Qtest/(2*sigma2)+Np*D/2*log(sigma2);
    
    if (abs(Q-Qtest) > 1e-13)
        fprintf('something is wrong\n');
    end
    %}
end