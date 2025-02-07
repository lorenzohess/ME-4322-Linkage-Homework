classdef Link < handle
    properties (SetAccess = private)
        length % meters
        mass
        weight
        mmi
        com
        joints % list of Joint
        groundJoint
        nonGroundJoints = [];
        num % integer ID, e.g. 1
        comVectorsStruct
        comCoords
    end

    properties
        angle % degrees
        angularVelocity = 0; % initial
        angularAcceleration = 0; % initial
        symAngularVelocity sym
        symAngularAcceleration sym
        symRx
        symRy
    end

    methods
        % Constructor
        function obj = Link(num, length, mass, mmi, joints)
            obj.num = num;
            obj.length = length;
            obj.mass = mass;
            obj.weight = mass * 9.81;
            obj.mmi = mmi;
            obj.joints = joints;
            obj.angle = obj.initialAngle();
            obj.assignGround();
            obj.comCoords = obj.computeCOMCoords();
            obj.comVectorsStruct = obj.computeJointToCOMVectors();

            omega = sym("omega" + num2str(obj.num));
            obj.symAngularVelocity = omega;

            alpha = sym("alpha" + num2str(obj.num));
            obj.symAngularAcceleration = alpha;

            % Rx = sym("Rx" + num2str(obj.num));
        end

        function comVector = jointToCOMVector(obj, joint)
            comVector = obj.comVectorsStruct.(joint.name);
        end

        function comCoords = computeCOMCoords(obj)
            % HACK: check if grounded
            % Take one joint coords and add other joint coords / 2
            if length(obj.groundJoint) > 0
                disp(obj.num)
                comCoords = [obj.groundJoint.x obj.groundJoint.y 0] + [obj.nonGroundJoints(1).x / 2, obj.nonGroundJoints(1).y / 2, 0]
            else
                disp(obj.num)
                comCoords = [obj.nonGroundJoints(1).x obj.nonGroundJoints(1).y 0] + [obj.nonGroundJoints(2).x / 2 obj.nonGroundJoints(2).y / 2 0]
            end
            % HACK: if ternary, i.e. CDE, use hardcoded from Onshape
            if 3 == length(obj.joints)
                disp(obj.num)
                % jD + distance 1.2463 away => COM of CDE at (0.32, 1.3)
                comCoords = [0.32 1.3 0];
            end
            % HACK: if FG, use gripper as one endpoint
            for joint = obj.joints
                if joint.name == "G"
                    comCoords = [obj.groundJoint.x obj.groundJoint.y 0] + [-1.225 / 2, 3.450 / 2, 0]
                end
            end
        end

        function comVectorsStruct = computeJointToCOMVectors(obj)
            comVectorsStruct = struct();
            for joint = obj.joints
                comVectorsStruct.(joint.name) = obj.comCoords - [joint.getCoords() 0];
            end
        end

        function printJointCoords(obj)
            for joint = obj.joints
                disp("Joint " + joint.name + ": " + joint.x + ", " + joint.y)
            end
        end

        function v = getCurrentAngularVelocityVector(obj)
            v = [0 0 obj.angularVelocity];
        end

        function symAngularVelocityVector = getSymAngularVelocityVector(obj)
            symAngularVelocityVector = [0 0 obj.symAngularVelocity];
        end

        function symAngularAccelerationVector = getSymAngularAccelerationVector(obj)
            symAngularAccelerationVector = [0 0 obj.symAngularAcceleration];
        end

        function velocity = getSymVelocity(obj, jTail, jHead)
        % Assumes position vector goes from tail ground joint to head non-ground
        % joint, or if neither joint is grounded, then from tail joints(1) to
        % head joints(2), unless headJoint is given, in which head is headJoint.
            if 1 == nargin % no args, ground -> non-ground, i.e. binary link
                jTail = obj.groundJoint;
                jHead = obj.nonGroundJoints(1);
            elseif 2 == nargin % jTail arg, ground -> second non-ground, i.e. ternary link
                jTail = obj.groundJoint;
                jHead = obj.nonGroundJoints(2);
            else % both jTail and jHead arg
            end
            velocity = cross(obj.getSymAngularVelocityVector(),...
                             [obj.jointToJointVector(jHead, jTail)]);
        end

        function velocity = getCurrentVelocity(obj, jTail, jHead)
        % Assumes position vector goes from tail ground joint to head non-ground
        % joint, or if neither joint is grounded, then from tail joints(1) to
        % head joints(2), unless headJoint is given, in which head is headJoint.
            if 1 == nargin % no args, ground -> non-ground, i.e. binary link
                jTail = obj.groundJoint;
                jHead = obj.nonGroundJoints(1);
            elseif 2 == nargin % jTail arg, ground -> second non-ground, i.e. ternary link
                jTail = obj.groundJoint;
                jHead = obj.nonGroundJoints(2);
            else % both jTail and jHead arg
            end
            velocity = cross(obj.getCurrentAngularVelocityVector(),...
                             [obj.jointToJointVector(jHead, jTail)]);
        end

        function acceleration = getSymAcceleration(obj, jTail, jHead)
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
            normal = cross(obj.getSymAngularAccelerationVector(), [obj.jointToJointVector(jHead, jTail)]);
            tangential = cross(obj.getCurrentAngularVelocityVector(), obj.getCurrentVelocity(jTail, jHead));
            acceleration = normal + tangential;
        end

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
            % HACK to check if jHead is Joint or COM vector
            if isobject(jHead)
                vector = [jHead.x - jTail.x, jHead.y - jTail.y, Z];
            else
                vector = [jHead(1) - jTail.x, jHead(2) - jTail.y, Z];
            end
        end

        function distance = jointToJointDistance(obj, j1, j2)
            distance = sqrt((j1.x - j2.x)^2 + (j1.y - j2.y)^2);
        end
    end
end
