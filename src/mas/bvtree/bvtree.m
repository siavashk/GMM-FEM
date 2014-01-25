% Thanks to Oliver Wood and Ken Chatfield
% http://www.mathworks.com/matlabcentral/newsreader/view_thread/278243
% http://stackoverflow.com/questions/13842893/matlab-parfor-and-c-class-mex-wrappers-copy-constructor-required

classdef bvtree < handle
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
        function this = bvtree(pnts, elems, tol)
            if (nargin < 3 || isempty(tol))
                tol = -1;
            end
            this.cpp_handle_ = bvtree_build(pnts, elems, tol);
            % disp(['Initialized bvtree with handle: ' num2str(this.cpp_handle_)]);
        end
        % destructor
        function delete(this)
            bvtree_destroy(this.cpp_handle_);
            % disp(['Deleted bvtree with handle: ' num2str(this.cpp_handle_)]);
        end
        % class method
        function [eIdxs] = intersect_point(this, pnts, r)
            if (nargin<3 || isempty(r))
                r = 0;
            end
            eIdxs = bvtree_intersect_point(this.cpp_handle_, pnts, r);
            % disp(['Used bvtree with handle: ' num2str(this.cpp_handle_)]);
        end
    end
end