classdef Link
    properties (SetAccess = private)
        length % meters
        mass
        mmi
    end

    methods
        % Constructor
        function obj = Link(length)
            obj.length = length;
        end

        % Getter
        function len = get.length(obj)
            len = obj.length;
        end
    end
end
