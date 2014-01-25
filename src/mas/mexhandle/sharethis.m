% Thanks to Oliver Wood and Ken Chatfield
% http://www.mathworks.com/matlabcentral/newsreader/view_thread/278243
% http://stackoverflow.com/questions/13842893/matlab-parfor-and-c-class-mex-wrappers-copy-constructor-required

classdef ShareThis < handle
    properties(SetAccess=protected, Transient=true)
        cpp_handle_
    end
    methods(Static=true)
        function this = loadobj(a)
            this.cpp_handle_ = a.cpp_handle_;
            disp(['Load initialized encoder with handle: ' num2str(a.cpp_handle_)]);          
        end
    end
    methods
        function this = ShareThis(id)
            obj.cpp_handle_ = create_sharethis(id);
            disp(['Initialized ShareThis with handle: ' num2str(this.cpp_handle_)]);
        end
        % destructor
        function delete(this)
            destroy_sharethis(this.cpp_handle_);
            disp(['Deleted ShareThis with handle: ' num2str(this.cpp_handle_)]);
        end
        % class method
        function use(this)
            use_sharethis(this.cpp_handle_);
            disp(['Used ShareThis with handle: ' num2str(this.cpp_handle_)]);
        end
    end
end