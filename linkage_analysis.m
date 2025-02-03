clc; clear; hold on; axis equal; format long;
% clc; clear; format long;

jA = Joint("A", 1.4, 0.485, true, 'k.');
jB = Joint("B", 1.67, 0.99, false, 'r.');
jC = Joint("C", 0.255, 1.035, false, 'g.');
jD = Joint("D", 0.285, 0.055, true, 'k.');
jE = Joint("E", 0.195, 2.54, false, 'b.');
jF = Joint("F", -0.98, 2.57, false, 'c.');
jG = Joint("G", 0.05, 0.2, true, 'k.');

crank = Crank("1", 0.5726, 1, 1, [jA jB]);
l2 = Link("2", 1.4157, 1, 1, [jB jC]);
l3 = Link("3", 2.4866, 1, 1, [jC, jD, jE]);
l4 = Link("4", 1.1754, 1, 1, [jE, jF]);
l5 = Link("5", 2.5841, 1, 1, [jG, jF]);

% Pre-compute distance between jC and jE because computing during iteration
% yields incorrect position analysis (error builds up in position of C).
DISTANCE_JC_TO_JE = l3.jointToJointDistance(jC, jE);

% syms w2 w3 w4 w5;
% W1 = [0 0 crank.INPUT_ANGULAR_VELOCITY];
% W2 = [0 0 w2];
% W3 = [0 0 w3];
% W4 = [0 0 w4];
% W5 = [0 0 w5];

% Iterate
for i = crank.START_ANGLE:crank.STEP_ANGLE:crank.END_ANGLE
    % Update crank angle
    crank.incrementAngle(crank.STEP_ANGLE);

    % Compute position of Joint B
    c = crank.updateCoords();
    plot(c(1), c(2), jB.color);

    % Compute position of Joint C
    c = circleIntersection(jC, jB, jD, l2.length, [l3.jointToJointDistance(jD, jC)]);
    plot(c(1), c(2), jC.color);

    % Compute position of Joint E
    c = circleIntersection(jE, jC, jD, DISTANCE_JC_TO_JE, l3.length);
    plot(c(1), c(2), jE.color);

    % Compute position of Joint F
    c = circleIntersection(jF, jG, jE, l5.length, l4.length);
    plot(c(1), c(2), jF.color);

    loop1 = cross(crank.getAngularVelocityVector(), crank.jointToJointVector(jB, jA)) + cross(l2.getAngularVelocityVector(), l2.jointToJointVector(jC, jB)) + cross(l3.getAngularVelocityVector(), l3.jointToJointVector(jD, jC));
    loop2 = cross(l3.getAngularVelocityVector(), l3.jointToJointVector(jE, jD)) + cross(l4.getAngularVelocityVector(), l4.jointToJointVector(jF, jE)) + cross(l5.getAngularVelocityVector(), l5.jointToJointVector(jG, jF));
    soln = solve([loop1, loop2], [l2.angularVelocity, l3.angularVelocity, l4.angularVelocity, l5.angularVelocity]);
end

function newCoords = circleIntersection(joint, knownJoint1, knownJoint2, r1, r2)
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
