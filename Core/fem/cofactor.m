function [ A ] = cofactor( M )
%COFACTOR computes the cofactor matrix of M

    [n1, n2] = size(M);
    
    A = zeros([n2 n1]);

    for i=1:n1
        r = [1:(i-1),(i+1):n1];
        
        for j=1:n2
            c =[1:(j-1),(j+1):n2];
            s = 1-2*bitand(i+j,1); 
            A(i,j) = s*det(M(r,c));
        end
    end


end

