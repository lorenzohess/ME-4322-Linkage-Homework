classdef Linkage
    properties (SetAccess = private)
        jA
        jB
        jC
        jD
        jE
        jF
        jG

        crank % the input link
        l1
        l2
        l3
        l4
        l5
        % Pre-compute distance between jC and jE because computing during iteration
        % yields incorrect position analysis (error builds up in position of C).
        DISTANCE_JC_TO_JE = 1.506195538434502;
    end
    methods
        function obj = Linkage()
            obj.jA = Joint("A", 1.4, 0.485, true, 'k.');
            obj.jB = Joint("B", 1.67, 0.99, false, 'r.');
            obj.jC = Joint("C", 0.255, 1.035, false, 'g.');
            obj.jD = Joint("D", 0.285, 0.055, true, 'k.');
            obj.jE = Joint("E", 0.195, 2.54, false, 'b.');
            obj.jF = Joint("F", -0.98, 2.57, false, 'c.');
            obj.jG = Joint("G", 0.05, 0.2, true, 'k.');

            obj.l1 = Crank("1", 0.5726, 1, 1, [obj.jA obj.jB]);
            obj.l2 = Link("2", 1.4157, 1, 1, [obj.jB obj.jC]);
            obj.l3 = Link("3", 2.4866, 1, 1, [obj.jC, obj.jD, obj.jE]);
            obj.l4 = Link("4", 1.1754, 1, 1, [obj.jE, obj.jF]);
            obj.l5 = Link("5", 2.5841, 1, 1, [obj.jG, obj.jF]);
            obj.crank = obj.l1;
        end

        function analyze(obj)
            for i = obj.crank.START_ANGLE:obj.crank.STEP_ANGLE:obj.crank.END_ANGLE
                % Update crank angle
                obj.crank.incrementAngle(obj.crank.STEP_ANGLE);

                obj.updatePositions();
                obj.updateAngularVelocities();
            end
        end

        function newCoords = circleIntersection(obj, joint, knownJoint1, knownJoint2, r1, r2)
        % Compute the new position of joint relative to two known joints, knownJoint1 and knownJoint2,
        % which connect to joint with link1 and link2.
        %
        % Side Effect: updates the coordinates of joint.
            syms x y;
            circle1 = (x - knownJoint1.x)^2 + (y - knownJoint1.y)^2 == r1^2;
            circle2 = (x - knownJoint2.x)^2 + (y - knownJoint2.y)^2 == r2^2;
            soln = solve(circle1, circle2);

            % Choose point closer to original
            newX1 = double(soln.x(1));
            newY1 = double(soln.y(1));
            newX2 = double(soln.x(2));
            newY2 = double(soln.y(2));

            distance1 = sqrt((joint.x - newX1)^2 + (joint.y - newY1)^2);
            distance2 = sqrt((joint.x - newX2)^2 + (joint.y - newY2)^2);

            if distance1 < distance2
                joint.setCoords(newX1, newY1);
                newCoords = [newX1, newY1];
            else % even when distance1 = distance2, can pick distance2
                joint.setCoords(newX2, newY2);
                newCoords = [newX2, newY2];
            end
        end

        function updatePositions(obj)
        % Compute position of Joint B
            c = obj.crank.updateCoords();
            plot(c(1), c(2), obj.jB.color);

            % Compute position of Joint C
            c = obj.circleIntersection(obj.jC, obj.jB, obj.jD, obj.l2.length,...
                                       [obj.l3.jointToJointDistance(obj.jD, obj.jC)]);
            plot(c(1), c(2), obj.jC.color);

            % Compute position of Joint E
            c = obj.circleIntersection(obj.jE, obj.jC, obj.jD, obj.DISTANCE_JC_TO_JE, obj.l3.length);
            plot(c(1), c(2), obj.jE.color);

            % Compute position of Joint F
            c = obj.circleIntersection(obj.jF, obj.jG, obj.jE, obj.l5.length, obj.l4.length);
            plot(c(1), c(2), obj.jF.color);
        end

        function updateAngularVelocities(obj)
            loop1 = cross(obj.crank.getAngularVelocityVector(), obj.crank.jointToJointVector(obj.jB, obj.jA)) +...
                    cross(obj.l2.getAngularVelocityVector(), obj.l2.jointToJointVector(obj.jC, obj.jB)) +...
                    cross(obj.l3.getAngularVelocityVector(), obj.l3.jointToJointVector(obj.jD, obj.jC));

            loop2 = cross(obj.l3.getAngularVelocityVector(), obj.l3.jointToJointVector(obj.jE, obj.jD)) +...
                    cross(obj.l4.getAngularVelocityVector(), obj.l4.jointToJointVector(obj.jF, obj.jE)) +...
                    cross(obj.l5.getAngularVelocityVector(), obj.l5.jointToJointVector(obj.jG, obj.jF));

            soln = solve([loop1, loop2], [obj.l2.angularVelocity, obj.l3.angularVelocity,...
                                          obj.l4.angularVelocity, obj.l5.angularVelocity]);
        end

    end
end
