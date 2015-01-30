function d = cpd_dice( v1, f1, v2, f2, rx, ry, rz )
%CPD_DICE Summary of this function goes here
%   Calculates the dice coefficient for two surfaces
%   v1,f1    vertices and faces of the first surface
%   v2,f2    vertices and faces of the second surface
%   rx,ry,rz sampling rates along each prinicipal direction
%   d        dice coefficient; d=2*Intersection(A,B)/(A+B)

% Calculate the bounding box of the collection of points first
tic;
X = [v1;v2];
[R, c, w] = tight_box(X);
w = w/2;    % change to half-widths

% Discretize the bounding box and count the number of points that are in
% each surface
sp = [2*w(1)/(rx-1), 2*w(2)/(ry-1), 2*w(3)/(rz-1)];
x1 = -w(1):sp(1):w(1);
x2 = -w(2):sp(2):w(2);
x3 = -w(3):sp(3):w(3);

[x1g,x2g,x3g] = meshgrid(x1, x2, x3);

xg = [reshape(x1g,1,[])' reshape(x2g,1,[])' reshape(x3g,1,[])'];

% Count the number of points in the each and also in the intersection of
% the two
v1 = bsxfun(@minus, v1, c);
v1 = v1*R';

v2 = bsxfun(@minus, v2, c);
v2 = v2*R';

% plot points
% clf
% patch('Vertices',v1,'Faces',f1,'FaceColor','r','FaceAlpha',.2);
% hold on;
% patch('Vertices',v2,'Faces',f2,'FaceColor','b','FaceAlpha',.2);
% plot3(xg(:,1), xg(:,2), xg(:,3), '.k','MarkerSize', 2);
% drawnow;

c1 = inpolyhedron(f1,v1,xg);
c2 = inpolyhedron(f2,v2,xg);

% flip zeroes and ones
c_int = c1.*c2;

% p1 = xg(c1==1,:);
% p2 = xg(c2==1,:);
% clf;
% plot3(p1(:,1), p1(:,2), p1(:,3), '.r','MarkerSize', 2);
% hold on;
% plot3(p2(:,1), p2(:,2), p2(:,3), 'ob','MarkerSize', 2);
% drawnow;

d = 2*sum(c_int)/(sum(c1)+sum(c2));

end

