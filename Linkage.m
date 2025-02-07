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

        Wart = 200; % Newtons
        artifactVector = [-1.225 6.830 0];

        plots
    end
    methods
        function obj = Linkage(plots)
            obj.jA = Joint("A", 1.4, 0.485, true, 'k.');
            obj.jB = Joint("B", 1.67, 0.99, false, 'r.');
            obj.jC = Joint("C", 0.255, 1.035, false, 'g.');
            obj.jD = Joint("D", 0.285, 0.055, true, 'k.');
            obj.jE = Joint("E", 0.195, 2.54, false, 'b.');
            obj.jF = Joint("F", -0.98, 2.57, false, 'c.');
            obj.jG = Joint("G", 0.05, 0.2, true, 'k.');

            obj.l1 = Crank("1", 0.5726, 1, 1, [obj.jA obj.jB]);
            obj.l2 = Link("2", 1.4157, 1, 1, [obj.jB obj.jC]);
            obj.l3 = Link("3", 2.4866, 1, 1, [obj.jD, obj.jC, obj.jE]);
            obj.l4 = Link("4", 1.1754, 1, 1, [obj.jE obj.jF]);
            obj.l5 = Link("5", 2.5841, 1, 1, [obj.jG obj.jF]);
            obj.crank = obj.l1;
            % obj.links = [obj.crank obj.l2 obj.l3 obj.l4 obj.l5];

            obj.plots = plots;
        end

        function analyzeStatics(obj)
            syms ax ay bx by cx cy dx dy ex ey fx fy gx gy Tin;

            ABax  = ax;
            ABbx  = bx;
            ABay  = ay;
            ABby  = by;
            BCbx  = -1 * bx;
            BCby  = -1 * by;
            BCcx  = cx;
            BCcy  = cy;
            EFex  = ex;
            EFey  = ey;
            EFfx  = fx;
            EFfy  = fy;
            FGfx  = -1 * fx;
            FGfy  = -1 * fy;
            FGgx  = gx;
            FGgy  = gy;
            CDEcx = -1 * cx;
            CDEcy = -1 * cy;
            CDEdx = dx;
            CDEdy = dy;
            CDEex = -1 * ex;
            CDEey = -1 * ey;

            ABx = ABax - ABbx == 0;
            BCx = BCbx - BCcx == 0;
            CDEx = CDEcx + CDEdx - CDEex == 0;
            EFx = EFex - EFfx == 0;
            FGx = FGfx - FGgx == 0;

            ABy = ABay - ABby - obj.crank.weight == 0;
            BCy = BCby - BCcy - obj.l2.weight == 0;
            CDEy = CDEcy + CDEdy - CDEey - obj.l3.weight == 0;
            EFy = EFey - EFfy - obj.l4.weight == 0;
            FGy = FGfy + FGgy - obj.l5.weight - obj.Wart == 0;

            % About A
            MAB = [0 0 Tin] + cross(obj.crank.jointToCOMVector(obj.jA), [0 -obj.crank.weight 0]) +...
                  cross(obj.crank.jointToJointVector(obj.jB, obj.jA), [-ABbx -ABby 0]) == 0;

            % About B
            MBC = cross(obj.l2.jointToCOMVector(obj.jB), [0 -obj.l2.weight 0]) +...
                  cross(obj.l2.jointToJointVector(obj.jC, obj.jB), [-BCcx -BCcy 0]) == 0;

            % About C
            MCDE = cross(obj.l3.jointToCOMVector(obj.jC), [0 -obj.l3.weight 0]) +...
                  cross(obj.l3.jointToJointVector(obj.jE, obj.jC), [-CDEex -CDEey 0]) + ...
                  cross(obj.l3.jointToJointVector(obj.jD, obj.jC), [CDEdx CDEdy 0]) == 0;

            % About E
            MEF = cross(obj.l4.jointToCOMVector(obj.jE), [0 -obj.l4.weight 0]) +...
                  cross(obj.l4.jointToJointVector(obj.jF, obj.jE), [-EFfx -EFfy 0]) == 0;

            % About G
            MFG = cross(obj.l5.jointToCOMVector(obj.jG), [0 -obj.l5.weight 0]) +...
                  cross(obj.l5.jointToJointVector(obj.jF, obj.jG), [FGfx FGfy 0]) +...
                  cross(obj.l5.jointToJointVector(obj.artifactVector, obj.jG), [0 -obj.Wart 0]) == 0;

            eqns = [ABx, BCx, CDEx, EFx, FGx, ABy, BCy, CDEy, EFy, FGy, MAB, MBC, MCDE, MEF, MFG];
            vars = [ax ay bx by cx cy dx dy ex ey fx fy gx gy Tin];

            soln = solve(eqns, vars);
            arrayfun(@double, table2array(struct2table(soln))')
            % [0 0 soln.Tin] - cross(obj.crank.jointToJointVector(obj.crank.com, obj.jA), [0 -obj.crank.weight 0]) -...
            %       cross(obj.crank.jointToJointVector(obj.jB, obj.jA), [soln.bx soln.by 0])
        end

        function analyzeDynamics(obj)
            for i = obj.crank.START_ANGLE:obj.crank.STEP_ANGLE:obj.crank.END_ANGLE
                % Update crank angle
                obj.crank.incrementAngle(obj.crank.STEP_ANGLE);

                obj.updatePositions();
                obj.updateAngularVelocities();
                obj.updateLinearVelocities();
                obj.updateAngularAccelerations();
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
            plot(obj.plots.jointPos, c(1), c(2), obj.jB.color)

            % Compute position of Joint C
            c = obj.circleIntersection(obj.jC, obj.jB, obj.jD, obj.l2.length,...
                                       [obj.l3.jointToJointDistance(obj.jD, obj.jC)]);
            plot(obj.plots.jointPos, c(1), c(2), obj.jC.color);

            % Compute position of Joint E
            c = obj.circleIntersection(obj.jE, obj.jC, obj.jD, obj.DISTANCE_JC_TO_JE, obj.l3.length);
            plot(obj.plots.jointPos, c(1), c(2), obj.jE.color);

            % Compute position of Joint F
            c = obj.circleIntersection(obj.jF, obj.jG, obj.jE, obj.l5.length, obj.l4.length);
            plot(obj.plots.jointPos, c(1), c(2), obj.jF.color);
        end

        function updateAngularVelocities(obj)
            loop1 = obj.crank.getSymVelocity() + obj.l2.getSymVelocity(obj.jB, obj.jC) +...
                    obj.l3.getSymVelocity(obj.jC, obj.jD);
            loop2 = obj.l3.getSymVelocity(obj.jE) + obj.l4.getSymVelocity(obj.jE, obj.jF) +...
                    obj.l5.getSymVelocity(obj.jF, obj.jG);

            soln = solve([loop1, loop2], [obj.l2.symAngularVelocity, obj.l3.symAngularVelocity,...
                                          obj.l4.symAngularVelocity, obj.l5.symAngularVelocity]);
            obj.l2.angularVelocity = soln.omega2;
            obj.l3.angularVelocity = soln.omega3;
            obj.l4.angularVelocity = soln.omega4;
            obj.l5.angularVelocity = soln.omega5;
            obj.plotLinkAngularVelocities();
        end

        function updateLinearVelocities(obj)
        end

        function updateAngularAccelerations(obj)
            loop1 = obj.crank.getSymAcceleration() + obj.l2.getSymAcceleration(obj.jB, obj.jC) + obj.l3.getSymAcceleration(obj.jC, obj.jD);
            loop2 = obj.l3.getSymAcceleration(obj.jE) + obj.l4.getSymAcceleration(obj.jE, obj.jF) + obj.l5.getSymAcceleration(obj.jF, obj.jG);

            soln = solve([loop1, loop2], [obj.l2.symAngularAcceleration, obj.l3.symAngularAcceleration,...
                                          obj.l4.symAngularAcceleration, obj.l5.symAngularAcceleration]);
            obj.l2.angularAcceleration = soln.alpha2;
            obj.l3.angularAcceleration = soln.alpha3;
            obj.l4.angularAcceleration = soln.alpha4;
            obj.l5.angularAcceleration = soln.alpha5;
            obj.plotLinkAngularAccelerations();
        end

        function plotLinkAngularVelocities(obj)
            plot(obj.plots.linkAngVel, obj.crank.angle, obj.l2.angularVelocity, 'r.', 'DisplayName', "Link " + num2str(obj.l2.num))
            plot(obj.plots.linkAngVel, obj.crank.angle, obj.l3.angularVelocity, 'g.', 'DisplayName', "Link " + num2str(obj.l3.num))
            plot(obj.plots.linkAngVel, obj.crank.angle, obj.l4.angularVelocity, 'b.', 'DisplayName', "Link " + num2str(obj.l4.num))
            plot(obj.plots.linkAngVel, obj.crank.angle, obj.l5.angularVelocity, 'c.', 'DisplayName', "Link " + num2str(obj.l5.num))
        end

        function plotLinkAngularAccelerations(obj)
            plot(obj.plots.linkAngAccel, obj.crank.angle, obj.l2.angularAcceleration, 'r.', 'DisplayName', "Link " + num2str(obj.l2.num))
            plot(obj.plots.linkAngAccel, obj.crank.angle, obj.l3.angularAcceleration, 'g.', 'DisplayName', "Link " + num2str(obj.l3.num))
            plot(obj.plots.linkAngAccel, obj.crank.angle, obj.l4.angularAcceleration, 'b.', 'DisplayName', "Link " + num2str(obj.l4.num))
            plot(obj.plots.linkAngAccel, obj.crank.angle, obj.l5.angularAcceleration, 'c.', 'DisplayName', "Link " + num2str(obj.l5.num))
        end
    end
end
