classdef Crank < Link
    properties (SetAccess = private)
        % Define iteration start,stop,step
        START_ANGLE = 1; % degree
        END_ANGLE = 360; % degree
        STEP_ANGLE = 1; % degree
        INPUT_ANGULAR_VELOCITY = 21.25; % rad/s
        INPUT_ANGULAR_ACCELERATION = 0; % rad/s/s
    end

    methods
        function obj = Crank(num, length, mass, mmi, joints, comCoords)
            obj@Link(num, length, mass, mmi, joints, comCoords);
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
