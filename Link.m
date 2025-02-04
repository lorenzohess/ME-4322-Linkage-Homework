classdef Link < handle
    properties (SetAccess = private)
        length % meters
        mass
        mmi
        joints % list of Joint
        groundJoint
        nonGroundJoints = [];
        num % integer ID, e.g. 1
    end

    properties
        angle % degrees
        angularVelocity sym % symbolic
    end

    methods
        % Constructor
        function obj = Link(num, length, mass, mmi, joints)
            obj.num = num;
            obj.length = length;
            obj.mass = mass;
            obj.mmi = mmi;
            obj.joints = joints;
            obj.angle = obj.initialAngle();
            obj.assignGround();

            omega = sym("omega" + num2str(obj.num));
            obj.angularVelocity = omega;
        end

        % Getters
        function len = get.length(obj)
            len = obj.length;
        end

        function nonGroundJoints = get.nonGroundJoints(obj)
            nonGroundJoints = obj.nonGroundJoints;
        end

        function printJointCoords(obj)
            for joint = obj.joints
                disp("Joint " + joint.name + ": " + joint.x + ", " + joint.y)
            end
        end

        function angularVelocityVector = getAngularVelocityVector(obj)
            angularVelocityVector = [0 0 obj.angularVelocity];
        end

        function velocity = getVelocity(obj, jTail, jHead)
        % Assumes position vector goes from tail ground joint to head non-ground
        % joint, or if neither joint is grounded, then from tail joints(1) to
        % head joints(2), unless headJoint is given, in which head is headJoint.
            % disp("Link: " + obj.num)
            if 1 == nargin % no args, ground -> non-ground, i.e. binary link
                jTail = obj.groundJoint;
                jHead = obj.nonGroundJoints(1);
            elseif 2 == nargin % jTail arg, ground -> second non-ground, i.e. ternary link
                jTail = obj.groundJoint;
                jHead = obj.nonGroundJoints(2);
            else % both jTail and jHead arg
            end
            velocity = cross(obj.getAngularVelocityVector(),...
                             [obj.jointToJointVector(jHead, jTail)]);
        end

        % Methods
        function assignGround(obj)
        % Doesn't handle more than one ground joint per link
            for joint = obj.joints
                if (joint.ground)
                    obj.groundJoint = joint;
                else
                    obj.nonGroundJoints = [obj.nonGroundJoints joint];
                end
            end
        end

        function angle = initialAngle(obj)
            deltaY = obj.joints(2).y - obj.joints(1).y;
            deltaX = obj.joints(2).x - obj.joints(1).x;
            angle = rad2deg(atan2(deltaY, deltaX));
        end

        function newAngle = incrementAngle(obj, value)
            obj.angle = obj.angle + value;
            % disp("New angle: " + obj.angle)
        end

        function vector = jointToJointVector(obj, jHead, jTail)
            Z = 0;
            vector = [jHead.x - jTail.x, jHead.y - jTail.y, Z];
        end

        function distance = jointToJointDistance(obj, j1, j2)
            distance = sqrt((j1.x - j2.x)^2 + (j1.y - j2.y)^2);
        end
    end
end
