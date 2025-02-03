classdef Link < handle
    properties (SetAccess = private)
        length % meters
        mass
        mmi
        joints % list of Joint
        groundJoint
    end

    properties
        angle % degrees
    end

    methods
        % Constructor
        function obj = Link(length, mass, mmi, joints)
            obj.length = length;
            obj.mass = mass;
            obj.mmi = mmi;
            obj.joints = joints;
            obj.angle = obj.initialAngle();
            obj.checkForGround();
        end

        % Getters
        function len = get.length(obj)
            len = obj.length;
        end

        function printJointCoords(obj)
            for joint = obj.joints
                disp("Joint " + joint.name + ": " + joint.x + ", " + joint.y)
            end
        end

        % Methods
        function checkForGround(obj)
            for joint = obj.joints
                if (joint.ground)
                    obj.groundJoint = joint;
                    return;
                end
            end
        end

        function angle = initialAngle(obj)
            deltaY = obj.joints(2).y - obj.joints(1).y;
            deltaX = obj.joints(2).x - obj.joints(1).x;
            angle = rad2deg(atan2(deltaY, deltaX));
        end

        function newAngle = updateAngle(obj)
            obj.angle = obj.angle + 1;
            disp("New angle: " + obj.angle)
        end

        function distance = jointToJointDistance(obj, j1, j2)
            distance = sqrt((j1.x - j2.x)^2 + (j1.y - j2.y)^2);
        end
    end
end
