function make_csg_mex(target)
%MAKE_CSG_MEX Makes the CSG interfaces

MATLAB_ROOT=matlabroot;
MATLAB_ROOT=strrep(MATLAB_ROOT,'\','/');

MEX_EXT=mexext;
archstr = computer('arch');
MATLAB_BINDIR=[MATLAB_ROOT,'/bin/',archstr];

if (nargin < 1 || isempty(target))
    target = 'mex';
end

MATLAB_DEFINES = ['MATLAB_ROOT="\"',MATLAB_ROOT,'\"" MAX_EXT=',MEX_EXT, ...
    ' MATLAB_BINDIR="',MATLAB_BINDIR,'"'];
% MATLAB_DEFINES='';

make_command = ['make ',target,' ', MATLAB_DEFINES];
system(make_command,'-echo');
system(['make cleanup ', MATLAB_DEFINES]);

end