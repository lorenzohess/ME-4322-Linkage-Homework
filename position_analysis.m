% clc; clear; hold on;
clc; clear;

jA = Joint("A", 1.4, 0.485, true);
jB = Joint("B", 1.67, 0.99, false);
jC = Joint("C", 0.255, 1.035, false);
jD = Joint("D", 0.285, 0.055, true);
jE = Joint("E", 0.195, 2.54, false);
jF = Joint("F", -0.98, 2.57, false);
jG = Joint("G", 0.05, 0.2, true);

crank = Crank(0.5726, 1, 1, [jA jB]);
l2 = Link(1.4157, 1, 1, [jB jC]);
l3 = Link(2.4866, 1, 1, [jC, jD, jE]);
l4 = Link(1.1754, 1, 1, [jE, jF]);
l5 = Link(2.5841, 1, 1, [jG, jF]);

for i = 1:360
    % Update crank angle
    crank.updateAngle();

    % Update position of link 1
    crank.updateCoords();
    % plot(b(1), b(2), '.');
    crank.printJointCoords()

    % Compute position of link 2
    circleIntersection(jC, jB, jD, l2.length, [l3.jointToJointDistance(jD, jC)]);
    l2.printJointCoords()

    % Update position of link 2
end

% Compute the new position of joint relative to two known joints, joint1 and joint2,
% which connect to joint with link1 and link2.
function newCoords = circleIntersection(joint, joint1, joint2, r1, r2)
    syms x y;
    circle1 = (x - joint1.x)^2 + (y - joint1.y)^2 == r1^2;
    circle2 = (x - joint2.x)^2 + (y - joint2.y)^2 == r2^2;
    soln = solve(circle1, circle2);

    % Choose point closer to original
    newX1 = double(soln.x(1));
    newY1 = double(soln.y(1));
    newX2 = double(soln.x(2));
    newY2 = double(soln.y(2));
    d1 = sqrt((joint.x - newX1)^2 + (joint.y - newY1)^2);
    d2 = sqrt((joint.x - newX2)^2 + (joint.y - newY2)^2);
    if d1 < d2
        joint.setCoords(newX1, newY1);
    else % even when d1 = d2, can pick d2
        joint.setCoords(newX2, newY2);
    end
end
