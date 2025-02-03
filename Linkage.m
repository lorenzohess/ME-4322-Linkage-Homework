classdef Linkage
    properties (SetAccess = private)
        links % list of Link
        joints % list of Joint
    end
    methods
        function obj = Linkage(links, joints)
            obj.links = links; % list of Link objects
            obj.joints = joints;
        end

        function positionSolver()
            for i = 1:360

            end
        end
    end
end
