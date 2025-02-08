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
        plotFormat
        hSeries
        seriesName
    end

    properties
        angle % degrees
        angularVelocity = 0; % initial
        angularAcceleration = 0; % initial
        COMAcceleration = [];
        symAngularVelocity sym
        symAngularAcceleration sym
        symRx
        symRy
    end

    methods
        % Constructor
        function obj = Link(num, length, mass, mmi, joints, comCoords, plotFormat)
            obj.num = num;
            obj.length = length;
            obj.mass = mass;
            obj.weight = mass * 9.81;
            obj.mmi = mmi;
            obj.joints = joints;
            obj.angle = obj.initialAngle();
            obj.assignGround();
            obj.comCoords = comCoords;
            obj.comVectorsStruct = obj.computeJointToCOMVectors();
            obj.plotFormat = plotFormat;

            omega = sym("omega" + num2str(obj.num));
            obj.symAngularVelocity = omega;

            alpha = sym("alpha" + num2str(obj.num));
            obj.symAngularAcceleration = alpha;
        end

        function generateLegendInfo(obj, ax)
            obj.seriesName = "Link " + obj.num;
            obj.hSeries = plot(ax, NaN, NaN, obj.plotFormat, 'DisplayName', obj.seriesName);
        end

        function comVector = jointToCOMVector(obj, joint)
            comVector = obj.comVectorsStruct.(joint.name);
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

        function v = getCurrentAngularAccelerationVector(obj)
            v = [0 0 obj.angularAcceleration];
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

        function velocity = getJointCurrentVelocity(obj, jHead)
        % HACK: Assumes position vector goes from tail ground joint to jHead.
        % If jHead isn't supplied, uses obj.nonGroundJoints(1) for jTail by
        % default, i.e.  supply jHead for ternary link.
            jTail = obj.groundJoint;
            if 1 == nargin
                jHead = obj.nonGroundJoints(1);
            end
            velocity = cross(obj.getCurrentAngularVelocityVector(),...
                             [obj.jointToJointVector(jHead, jTail)]);
        end

        function v = getCurrentVelocity(obj, jTail, jHead)
            velocity = cross(obj.getCurrentAngularVelocityVector(),...
                             [obj.jointToJointVector(jHead, jTail)]);
        end

        function acceleration = getSymAcceleration(obj, jTail, jHead)
        % HACK: Assumes position vector goes from tail ground joint to head
        % non-ground joint, or if neither joint is grounded, then from tail
        % joints(1) to head joints(2).
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

        function accJointToJointVector = computeJointToJointAcceleration(obj, jTail, jHead)
        % HACK: also used for COM to Joint relative acceleration
            jointToJointVector = obj.jointToJointVector(jHead, jTail);
            angVel = obj.getCurrentAngularVelocityVector();

            accTangential = cross(obj.getCurrentAngularAccelerationVector(), jointToJointVector);
            accNormal = cross(angVel,...
                            cross(angVel, jointToJointVector));
            accJointToJointVector = accTangential + accNormal;
        end

        function accCOMToJointVector = computeCOMToJointAcceleration(obj, joint)
        % Compute accel of COM wrto. joint
            jointToCOM = obj.jointToCOMVector(joint);
            angVel = obj.getCurrentAngularVelocityVector();

            accTangential = cross(obj.getCurrentAngularAccelerationVector(), jointToCOM);
            accNormal = cross(angVel,...
                            cross(angVel, jointToCOM));
            accCOMToJointVector = accTangential + accNormal;
        end

        function accCOMGroundedVector = computeCOMGroundedAcceleration(obj)
        % Compute accel of COM wrto. ground joint
            accCOMGroundedVector = obj.computeCOMToJointAcceleration(obj.groundJoint);
        end

        function assignGround(obj)
        % HACK: Doesn't handle more than one ground joint per link
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
            disp("Crank angle: " + obj.angle)
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
