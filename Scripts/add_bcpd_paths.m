% get root of current file
root = fullfile(fileparts(mfilename('fullpath')),'../');

p_generated = genpath([root '/Core']);
addpath(p_generated);
addpath([root '/IO']);
addpath([root '/Data']);
addpath([root '/Scripts']);

p_generated = genpath([root '/ThirdParty/tetgen1.4.3/bin']);
addpath(p_generated);

p_generated = genpath([root '/ThirdParty/maslib/bin']);
addpath(p_generated);

clear p_generated;
