disp('Check this file if the project and lib paths are set correctly before running!');


%% Set project path and compile information

%project_path = fullfile(pwd,'../../');
project_path = fullfile('/home/navarro/ThesisAlvaro-TEB/Navarro/Navarro/');

% mac:
%project_path = fullfile(pwd,'../../');

include_path_main = fullfile(project_path,'include');
include_path_eigen = fullfile(project_path,'extern','eigen3'); % support Eigen code in the interface.

library_path = fullfile(project_path,'build','lib','Debug'); % !!! Make sure to compile library in RELEASE Mode!
% win
library_name = 'teb_package'; %.lib
% mac
%library_name = 'libteb_package.a';

% integrator_system.h is placed in the current folder, therefore we do not need to add it.

%% Compile MEX function

%mex(['-I' include_path_main],...
 %   ['-I' include_path_eigen],...
  %  ['-L' library_path],...
   % ['-l' library_name],...
    %'integrator_system_mex.cpp');

%% Compile S-Function function

mex(['-I' include_path_main],...
    ['-I' include_path_eigen],...
    ['-L' library_path],...
    ['-l' library_name],...
    'car_like_system_sfun.cpp');


%% Compile S-Function without existing package library directly from source
    
mex(['-I' include_path_main],...
['-I' include_path_eigen],...
'car_like_system_sfun.cpp', ... // or integrator_system_mex.cpp for the matlab interface
[project_path 'src\base\base_controller.cpp'], ...
[project_path 'src\base\base_solver_least_squares.cpp'], ...
[project_path 'src\base\workspaces.cpp'], ...
[project_path 'src\solver\solver_levenbergmarquardt_eigen_dense.cpp'], ...
[project_path 'src\solver\solver_levenbergmarquardt_eigen_sparse.cpp'] );
    
