classdef control_interface < handle
%control_interface - Wrapper class for MPC/TEB controllers (MEX/C++)
% ========================================================================
%
%  Description:
%    This class creates an interface to use the MPC/TEB controllers
%    designed using the following C++ package:
%    http://www.rst.e-technik.tu-dortmund.de/SoftwareDokumentation/TimedElasticBand/html/
%   
%    This interface class calls the mex function contained in the compiled integrator_system_mex.cpp   
%    with different commands. Therefore it is now specialized for the integrator system with 2
%    states and a single control input.
%    Due to the simple and modular mex-intercace, this class is easy adaptable to other
%    controller and system types.
%    See the C++ source for more information.
%
%  Example:
%    ctrl = integrator_system_mex; % Construct class
%    u = ctrl.step([0;0], [1;0])   % Perform a single optimization step
%    ctrl.plotTEB;                 % Visualize states and control inputs
%    ctrl.delete;                  % Delete C++ objects from the heap
%
%  Known Bugs:
%    n/a
%
%  Authors:
%    <a href="matlab:web('mailto:christoph.roesmann@tu-dortmund.de')">Christoph Rösmann</a>
%    Original source: http://www.mathworks.com/matlabcentral/fileexchange/38964-example-matlab-class-wrapper-for-a-c++-class
%    (See cpproot/include/teb_package/utilities/matlab_class_handle.hpp for source details and licensing)
%
%  For research use only
% =======================================================================

    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
    end
    
    methods
        % Constructor - Create a new C++ class instance 
        function this = control_interface(varargin)
            this.objectHandle = integrator_system_mex('new', varargin{:});
        end
        
        % Destructor - Destroy the C++ class instance
        function delete(this)
            integrator_system_mex('delete', this.objectHandle);
        end

        % Call MPC step
        function u = step(this, x0,xf)
            if length(x0)~=2 || length(xf)~=2
                error('Vectors x0 and xf must be of length 2');
            end
            u = integrator_system_mex('step', this.objectHandle, x0(:), xf(:));
        end
        
        % Get TEB matrix containing time vector, states and control inputs of the optimization
        % Row 1: Time, Row 2-3: States, Row 4: Control input
        function teb = getTEBMatrix(this)
            teb = integrator_system_mex('tebmatrix', this.objectHandle);
        end

        % Plot TEB
        function plotTEB(this)
           teb = getTEBMatrix(this);
           figure(1);
        
           subplot(2,2,1);
           plot(teb(1,:),teb(2,:));
           grid on
           xlabel('t [s]');
           ylabel('x_1(t)');
           title('States after optimization');
           
           subplot(2,2,3);
           plot(teb(1,:),teb(3,:));
           grid on
           xlabel('t [s]');
           ylabel('x_2(t)');
           
           subplot(2,2,2);
           plot(teb(1,:),teb(4,:));
           grid on
           xlabel('t [s]');
           ylabel('u(t)');
           title('Control inputs after optimization');
        end
        
    end
end