% Thanks to Oliver Wood and Ken Chatfield
% http://www.mathworks.com/matlabcentral/newsreader/view_thread/278243
% http://stackoverflow.com/questions/13842893/matlab-parfor-and-c-class-mex-wrappers-copy-constructor-required

classdef smesh_bvtree < handle
    properties(SetAccess=protected, Transient=true)
        cpp_handle_
    end
    methods(Static=true)
        function this = loadobj(a)
            this.cpp_handle_ = a.cpp_handle_;
            % disp(['Load initialized encoder with handle: ' num2str(a.cpp_handle_)]);          
        end
    end
    methods
        function this = smesh_bvtree(pnts, faces, tol)
            if (nargin < 3 || isempty(tol))
                tol = -1;
            end
            this.cpp_handle_ = smesh_bvtree_build(pnts, faces, tol);
            % disp(['Initialized bvtree with handle: ' num2str(this.cpp_handle_)]);
        end

        % destructor
        function delete(this)
            smesh_bvtree_destroy(this.cpp_handle_);
            % disp(['Deleted bvtree with handle: ' num2str(this.cpp_handle_)]);
        end

        % class method
        function [fIdxs, nearest] = nearest_polygon(this, pnts)
            [fIdxs, nearest] = smesh_bvtree_nearest_polygon(this.cpp_handle_, pnts);
            % disp(['Used bvtree with handle: ' num2str(this.cpp_handle_)]);
        end

        % class method
        function [inside] = is_inside(this, pnts, tol, maxRetries)
			if (nargin < 3 || isempty(tol))
				tol = -1;
			end
			if (nargin < 4 || isempty(maxRetries))
				maxRetries = 100;
			end
            [inside] = smesh_bvtree_is_inside(this.cpp_handle_, pnts, tol, maxRetries);
            % disp(['Used bvtree with handle: ' num2str(this.cpp_handle_)]);
        end
    end
end
