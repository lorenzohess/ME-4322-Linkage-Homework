classdef Crank < Link
    properties (SetAccess = private)
        % Define iteration start,stop,step
        START_ANGLE = 1; % degree
        END_ANGLE = 360; % degree
        ANGULAR_DIRECTION = -1; % clockwise
        STEP_ANGLE = 1; % degree
        INPUT_ANGULAR_SPEED = 1.86;
        INPUT_ANGULAR_ACCELERATION = 0; % rad/s/s
        STEP
        INPUT_ANGULAR_VELOCITY
    end
    properties
        stepCounter = 0;
    end

    methods
        function obj = Crank(num, length, mass, mmi, joints, comCoords, plotFormat)
            obj@Link(num, length, mass, mmi, joints, comCoords, plotFormat);

            obj.STEP = obj.ANGULAR_DIRECTION * obj.STEP_ANGLE; % degree
            obj.INPUT_ANGULAR_VELOCITY = obj.ANGULAR_DIRECTION * obj.INPUT_ANGULAR_SPEED; % rad/s

            obj.angularVelocity = obj.INPUT_ANGULAR_VELOCITY;
            obj.symAngularVelocity = obj.INPUT_ANGULAR_VELOCITY;
            obj.angularAcceleration = obj.INPUT_ANGULAR_ACCELERATION;
            obj.symAngularAcceleration = obj.INPUT_ANGULAR_ACCELERATION;
        end

        function updatedCoords = updateCoords(obj)
            for joint = obj.nonGroundJoints
                newX = obj.groundJoint.x + obj.length * cos(deg2rad(obj.angle));
                newY = obj.groundJoint.y + obj.length * sin(deg2rad(obj.angle));
                joint.setCoords(newX, newY);
                updatedCoords = [newX, newY];
            end
        end
    end
end
