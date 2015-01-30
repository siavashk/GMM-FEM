function [ fem ] = createBoundingFem( pnts, margins, res )
%CREATEBOUNDINGFEM creates a hex fem beam that bounds a surface or a volume
%   fem = createBoundingFem(pnts, margins, res)
%           creates a beam that bounds the supplied bounds, with extra
%           space around the best-fit axes according to the supplied
%           margins
%   fem = createBoundingFem([], margins, res)
%           creates a beam that bounds a volume. This beam starts at
%           [margins(1) margins(3) margins(5)] and ends at 
%           [margins(2) margins(4) margins(6)] with a resolution of res.
%   pnts:    Nx3 set of 3D points
%   margins: either 1x1, 3x1, or 6x1 set of margin specifications
%            extra space around best-fit x,y,z axes
%            1x1: same value all around
%            1x3: x, y, z margin
%            1x6: -x,+x,-y,+y,-z,+z margin
%   res:     1x3 number of elements along each dimension
if (nargin < 2 || isempty(margins))
    margins = 0;
end
if (~isempty(pnts))
    [R, c, w] = tight_box(pnts);
    % correct for margin
    mm = zeros(6,1);
    if (length(margins) == 1)
        mm = ones(6,1)*margins;
    elseif (length(margins) == 3)
        mm = [margins(1), margins(1), margins(2), margins(2), margins(3), margins(3)];
    elseif (length(margins) ==6)
        mm = margins;
    end
    
    w(1) = w(1)+mm(1)+mm(2);
    w(2) = w(2)+mm(3)+mm(4);
    w(3) = w(3)+mm(5)+mm(6);
    
    c(1) = c(1) + mm(2)-mm(1);
    c(2) = c(2) + mm(4)-mm(3);
    c(3) = c(3) + mm(6)-mm(5);
else
    if (length(margins) ~= 6)
        error('Volume bound needs to be a 1x6 vector.');
    else
        R = eye(3);
        c = [(margins(2)+margins(1))/2 (margins(4)+margins(3))/2 (margins(6)+margins(5))/2];
        w = [margins(2)-margins(1) margins(4)-margins(3) margins(6)-margins(5)];
    end
end

fem = fem_model.createBeam(w, res, c, R);

end

