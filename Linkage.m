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
        links

        % Pre-compute distance between jC and jE because computing during iteration
        % yields incorrect position analysis (error builds up in position of C).
        DISTANCE_JC_TO_JE = 1.506195538434502;

        Wart = 200; % Newtons
        artifactVector = [-1.225 3.450 0];

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

            obj.l1 = Crank("1", 0.5726, 8.35, 0.266, [obj.jA obj.jB], [1.535 0.7375 0], 'k.');
            obj.crank = obj.l1;
            obj.l2 = Link("2", 1.4157, 20.2, 3.528, [obj.jB obj.jC], [0.9624 1.0125 0], 'r.');
            obj.l3 = Link("3", 2.4866, 34.85, 18.54, [obj.jD, obj.jC, obj.jE], [0.24 1.2975 0], 'g.');
            obj.l4 = Link("4", 1.1754, 16.82, 2.05, [obj.jE obj.jF], [-0.465 1.385 0], 'b.');
            obj.l5 = Link("5", 2.5841, 62.11, 103, [obj.jG obj.jF], [-0.3925 2.555 0], 'c.');
            obj.links = {obj.crank, obj.l2, obj.l3, obj.l4, obj.l5};

            obj.plots = plots;
        end

        function analyzeStatics(obj)
            syms ax ay bx by cx cy dx dy ex ey fx fy gx gy Tin;

            ABax  = ax;
            ABbx  = bx;
            ABay  = ay;
            ABby  = by;

            BCbx  = -1 * ABax;
            BCby  = -1 * ABay;
            BCcx  = cx;
            BCcy  = cy;

            CDEcx = -1 * BCcx;
            CDEcy = -1 * BCcy;
            CDEdx = dx;
            CDEdy = dy;
            CDEex = ex;
            CDEey = ey;

            EFex  = -1 * CDEex;
            EFey  = -1 * CDEey;
            EFfx  = fx;
            EFfy  = fy;

            FGfx  = -1 * EFfx;
            FGfy  = -1 * EFfy;
            FGgx  = gx;
            FGgy  = gy;

            % Force in X
            ABx = ABax - ABbx == 0;
            BCx = BCbx - BCcx == 0;
            CDEx = CDEcx - CDEdx - CDEex == 0;
            EFx = EFex - EFfx == 0;
            FGx = FGfx - FGgx == 0;

            % Force in Y
            ABy = -ABay + ABby - obj.crank.weight == 0;
            BCy = -BCby + BCcy - obj.l2.weight == 0;
            CDEy = -CDEcy + CDEdy - CDEey - obj.l3.weight == 0;
            EFy = -EFey + EFfy - obj.l4.weight == 0;
            FGy = -FGfy + FGgy - obj.l5.weight - obj.Wart == 0;

            % Moments
            % About A
            MAB = [0 0 -Tin] + cross(obj.crank.jointToCOMVector(obj.jA), [0 -obj.crank.weight 0]) +...
                  cross(obj.crank.jointToJointVector(obj.jB, obj.jA), [-ABbx ABby 0]) == 0;

            % About B
            MBC = cross(obj.l2.jointToCOMVector(obj.jB), [0 -obj.l2.weight 0]) +...
                  cross(obj.l2.jointToJointVector(obj.jC, obj.jB), [-BCcx BCcy 0]) == 0;

            % About C
            MCDE = cross(obj.l3.jointToCOMVector(obj.jC), [0 -obj.l3.weight 0]) +...
                  cross(obj.l3.jointToJointVector(obj.jE, obj.jC), [-CDEex CDEey 0]) + ...
                  cross(obj.l3.jointToJointVector(obj.jD, obj.jC), [-CDEdx CDEdy 0]) == 0;

            % About E
            MEF = cross(obj.l4.jointToCOMVector(obj.jE), [0 -obj.l4.weight 0]) +...
                  cross(obj.l4.jointToJointVector(obj.jF, obj.jE), [-EFfx -EFfy 0]) == 0;

            % About G
            MFG = cross(obj.l5.jointToCOMVector(obj.jG), [0 -obj.l5.weight 0]) +...
                  cross(obj.l5.jointToJointVector(obj.jF, obj.jG), [FGfx -FGfy 0]) +...
                  cross(obj.l5.jointToJointVector(obj.artifactVector, obj.jG), [0 -obj.Wart 0]) == 0;

            eqns = [ABx, BCx, CDEx, EFx, FGx, ABy, BCy, CDEy, EFy, FGy, MAB, MBC, MCDE, MEF, MFG];
            vars = [ax ay bx by cx cy dx dy ex ey fx fy gx gy Tin];

            soln = solve(eqns, vars);
            soln = arrayfun(@double, table2array(struct2table(soln))');
            soln_ax  = soln(1);
            soln_ay  = soln(2);
            soln_bx  = soln(3);
            soln_by  = soln(4);
            soln_cx  = soln(5);
            soln_cy  = soln(6);
            soln_dx  = soln(7);
            soln_dy  = soln(8);
            soln_ex  = soln(9);
            soln_ey  = soln(10);
            soln_fx  = soln(11);
            soln_fy  = soln(12);
            soln_gx  = soln(13);
            soln_gy  = soln(14);
            soln_Tin = soln(15);

            % Expected:
            expected_soln_ax  = 734.7777;
            expected_soln_ay  = 157.627;
            expected_soln_bx  = -734.7777;
            expected_soln_by  = -75.7135;
            expected_soln_dx  = -455.4051;
            expected_soln_dy  = 539.6962;
            expected_soln_ex  = -279.3726;
            expected_soln_ey  = -75.3692;
            expected_soln_cx  = -734.7777;
            expected_soln_cy  = 122.4485;
            expected_soln_fx  = -279.3726;
            expected_soln_fy  = 89.635;
            expected_soln_gx  = -279.3726;
            expected_soln_gy  = 898.9341;
            expected_soln_Tin = -339.5618;


            % obj.compareSolns("ax", soln_ax, expected_soln_ax)
            % obj.compareSolns("ay", soln_ay, expected_soln_ay)
            % obj.compareSolns("bx", soln_bx, expected_soln_bx)
            % obj.compareSolns("by", soln_by, expected_soln_by)
            % obj.compareSolns("cx", soln_cx, expected_soln_cx)
            % obj.compareSolns("cy", soln_cy, expected_soln_cy)
            % obj.compareSolns("dx", soln_dx, expected_soln_dx)
            % obj.compareSolns("dy", soln_dy, expected_soln_dy)
            % obj.compareSolns("ex", soln_ex, expected_soln_ex)
            % obj.compareSolns("ey", soln_ey, expected_soln_ey)
            % obj.compareSolns("fx", soln_fx, expected_soln_fx)
            % obj.compareSolns("fy", soln_fy, expected_soln_fy)
            % obj.compareSolns("gx", soln_gx, expected_soln_gx)
            % obj.compareSolns("gy", soln_gy, expected_soln_gy)
            % obj.compareSolns("Tin", soln_Tin, expected_soln_Tin)
        end

        function compareSolns(obj, s, mine, pmks)
            disp(s + " ==> Mine: " + num2str(mine) + "   PMKS: " + num2str(pmks))
        end

        function analyzeDynamics(obj)
            for i = obj.crank.START_ANGLE:obj.crank.STEP_ANGLE:obj.crank.END_ANGLE
                % Update crank angle
                obj.crank.incrementAngle(obj.crank.STEP);
                obj.crank.stepCounter = obj.crank.stepCounter + 1;

                obj.updatePositions();

                obj.updateAngularVelocities();
                obj.plotLinkAngularVelocities();

                obj.computeAndPlotLinearVelocities();

                obj.updateAngularAccelerations();
                obj.plotLinkAngularAccelerations();

                obj.updateCOMAccelerations();
                obj.plotCOMAccelerations()
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
            plot(obj.plots.linJointPos, c(1), c(2), obj.jB.color)

            % Compute position of Joint C
            c = obj.circleIntersection(obj.jC, obj.jB, obj.jD, obj.l2.length,...
                                       [obj.l3.jointToJointDistance(obj.jD, obj.jC)]);
            plot(obj.plots.linJointPos, c(1), c(2), obj.jC.color);

            % Compute position of Joint E
            c = obj.circleIntersection(obj.jE, obj.jC, obj.jD, obj.DISTANCE_JC_TO_JE, obj.l3.length);
            plot(obj.plots.linJointPos, c(1), c(2), obj.jE.color);

            % Compute position of Joint F
            c = obj.circleIntersection(obj.jF, obj.jG, obj.jE, obj.l5.length, obj.l4.length);
            plot(obj.plots.linJointPos, c(1), c(2), obj.jF.color);
        end

        function updateAngularVelocities(obj)
            loop1 = obj.crank.getSymVelocity() + obj.l2.getSymVelocity(obj.jB, obj.jC) +...
                    obj.l3.getSymVelocity(obj.jC, obj.jD) == 0;
            loop2 = obj.l3.getSymVelocity(obj.jE) + obj.l4.getSymVelocity(obj.jE, obj.jF) +...
                    obj.l5.getSymVelocity(obj.jF, obj.jG) == 0;

            soln = solve([loop1, loop2], [obj.l2.symAngularVelocity, obj.l3.symAngularVelocity,...
                                          obj.l4.symAngularVelocity, obj.l5.symAngularVelocity]);
            obj.l2.angularVelocity = soln.omega2;
            obj.l3.angularVelocity = soln.omega3;
            obj.l4.angularVelocity = soln.omega4;
            obj.l5.angularVelocity = soln.omega5;
        end

        function computeAndPlotLinearVelocities(obj)
            obj.l2.generateLegendInfo(obj.plots.linJointVelX);
            obj.l3.generateLegendInfo(obj.plots.linJointVelX);
            obj.l4.generateLegendInfo(obj.plots.linJointVelX);
            obj.l5.generateLegendInfo(obj.plots.linJointVelX);

            obj.l2.generateLegendInfo(obj.plots.linJointVelY);
            obj.l3.generateLegendInfo(obj.plots.linJointVelY);
            obj.l4.generateLegendInfo(obj.plots.linJointVelY);
            obj.l5.generateLegendInfo(obj.plots.linJointVelY);

            % jB
            jBlinVel = obj.crank.getCurrentVelocity(obj.jA, obj.jB);
            plot(obj.plots.linJointVelX, obj.crank.stepCounter, jBlinVel(1), obj.jB.color)
            plot(obj.plots.linJointVelY, obj.crank.stepCounter, jBlinVel(2), obj.jB.color)

            % jC
            jClinVel = obj.l3.getCurrentVelocity(obj.jB, obj.jC);
            plot(obj.plots.linJointVelX, obj.crank.stepCounter, jClinVel(1), obj.jC.color)
            plot(obj.plots.linJointVelY, obj.crank.stepCounter, jClinVel(2), obj.jC.color)

            % jE
            jElinVel = obj.l4.getCurrentVelocity(obj.jB, obj.jE);
            plot(obj.plots.linJointVelX, obj.crank.stepCounter, jElinVel(1), obj.jE.color)
            plot(obj.plots.linJointVelY, obj.crank.stepCounter, jElinVel(2), obj.jE.color)

            % jF
            jFlinVel = obj.l5.getCurrentVelocity(obj.jB, obj.jF);
            plot(obj.plots.linJointVelX, obj.crank.stepCounter, jFlinVel(1), obj.jF.color)
            plot(obj.plots.linJointVelY, obj.crank.stepCounter, jFlinVel(2), obj.jF.color)

            legend(obj.plots.linJointVelX, [obj.l2.hSeries, obj.l3.hSeries, obj.l4.hSeries, obj.l5.hSeries],...
                   {obj.l2.seriesName, obj.l3.seriesName, obj.l4.seriesName, obj.l5.seriesName}, 'Location', 'Best');

            legend(obj.plots.linJointVelY, [obj.l2.hSeries, obj.l3.hSeries, obj.l4.hSeries, obj.l5.hSeries],...
                   {obj.l2.seriesName, obj.l3.seriesName, obj.l4.seriesName, obj.l5.seriesName}, 'Location', 'Best');
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
        end

        function updateJointAccelerations(obj)
        end

        function updateCOMAccelerations(obj)
        % Ignore link 1, which COM accel = 0
            for link = [obj.l3 obj.l5]
                link.COMAcceleration = link.computeCOMGroundedAcceleration();
            end
            % HACK: do BC and EF hardcoded
            accCOMToB = obj.l2.computeCOMToJointAcceleration(obj.jB);
            accBToA = obj.crank.computeJointToJointAcceleration(obj.jA, obj.jB);
            obj.l2.COMAcceleration = accCOMToB + accBToA;

            accCOMToF = obj.l4.computeCOMToJointAcceleration(obj.jF);
            accFToG = obj.l5.computeJointToJointAcceleration(obj.jG, obj.jF);
            obj.l4.COMAcceleration = accCOMToF + accFToG;
        end

        %%% Plotting
        function plotLinkAngularVelocities(obj)
            obj.l2.generateLegendInfo(obj.plots.angLinkVel);
            obj.l3.generateLegendInfo(obj.plots.angLinkVel);
            obj.l4.generateLegendInfo(obj.plots.angLinkVel);
            obj.l5.generateLegendInfo(obj.plots.angLinkVel);

            % Empty points to make legend not give legend Series for each point plotted after
            plot(obj.plots.angLinkVel, obj.crank.stepCounter, obj.l2.angularVelocity, obj.l2.plotFormat)
            plot(obj.plots.angLinkVel, obj.crank.stepCounter, obj.l3.angularVelocity, obj.l3.plotFormat)
            plot(obj.plots.angLinkVel, obj.crank.stepCounter, obj.l4.angularVelocity, obj.l4.plotFormat)
            plot(obj.plots.angLinkVel, obj.crank.stepCounter, obj.l5.angularVelocity, obj.l5.plotFormat)

            legend(obj.plots.angLinkVel, [obj.l2.hSeries, obj.l3.hSeries, obj.l4.hSeries, obj.l5.hSeries],...
                   {obj.l2.seriesName, obj.l3.seriesName, obj.l4.seriesName, obj.l5.seriesName}, 'Location', 'Best');
        end

        function plotLinkAngularAccelerations(obj)
            obj.l2.generateLegendInfo(obj.plots.angLinkAccel);
            obj.l3.generateLegendInfo(obj.plots.angLinkAccel);
            obj.l4.generateLegendInfo(obj.plots.angLinkAccel);
            obj.l5.generateLegendInfo(obj.plots.angLinkAccel);

            plot(obj.plots.angLinkAccel, obj.crank.stepCounter, obj.l2.angularAcceleration, obj.l2.plotFormat)
            plot(obj.plots.angLinkAccel, obj.crank.stepCounter, obj.l3.angularAcceleration, obj.l3.plotFormat)
            plot(obj.plots.angLinkAccel, obj.crank.stepCounter, obj.l4.angularAcceleration, obj.l4.plotFormat)
            plot(obj.plots.angLinkAccel, obj.crank.stepCounter, obj.l5.angularAcceleration, obj.l5.plotFormat)

            legend(obj.plots.angLinkAccel, [obj.l2.hSeries, obj.l3.hSeries, obj.l4.hSeries, obj.l5.hSeries],...
                   {obj.l2.seriesName, obj.l3.seriesName, obj.l4.seriesName, obj.l5.seriesName}, 'Location', 'Best');
        end

        function plotCOMAccelerations(obj)
            obj.l2.generateLegendInfo(obj.plots.linCOMAccel);
            obj.l3.generateLegendInfo(obj.plots.linCOMAccel);
            obj.l4.generateLegendInfo(obj.plots.linCOMAccel);
            obj.l5.generateLegendInfo(obj.plots.linCOMAccel);

            plot(obj.plots.linCOMAccel, obj.crank.stepCounter, obj.l2.COMAcceleration(1), obj.l2.plotFormat)
            plot(obj.plots.linCOMAccel, obj.crank.stepCounter, obj.l3.COMAcceleration(1), obj.l3.plotFormat)
            plot(obj.plots.linCOMAccel, obj.crank.stepCounter, obj.l4.COMAcceleration(1), obj.l4.plotFormat)
            plot(obj.plots.linCOMAccel, obj.crank.stepCounter, obj.l5.COMAcceleration(1), obj.l5.plotFormat)

            legend(obj.plots.linCOMAccel, [obj.l2.hSeries, obj.l3.hSeries, obj.l4.hSeries, obj.l5.hSeries],...
                   {obj.l2.seriesName, obj.l3.seriesName, obj.l4.seriesName, obj.l5.seriesName}, 'Location', 'Best');
        end
    end
end
