clc; clear; hold on;

jA = Joint("A", 1.4, 0.485, true);
jB = Joint("B", 1.69, 0.99, false);
jC = Joint("C", 0.255, 1.035, false);
jD = Joint("D", 0.285, 0.055, true);
jE = Joint("E", 0.195, 2.54, false);
jF = Joint("F", -0.98, 2.57, false);
jG = Joint("G", 0.05, 0.2, true);

l1 = Link(0.5726, 1, 1, [jA jB]);
l2 = Link(1.4157, 1, 1, [jB jC]);
% l3 = Link(2.4866, 1, 1, [jC, jD, jE]);
% l4 = Link(1.1754, 1, 1, [jE, jF]);
% l5 = Link(2.5841, 1, 1, [jG, jF]);

for i = 1:360
    l1.updateAngle();
    c = l1.updateCoords()
    plot(c(1), c(2), '.');
    l1.printJointCoords()
end
% circleIntersection()

% Compute the new position of joint relative to two known joints, joint1 and joint2,
% which connect to joint with link1 and link2.
function [newX, newY] = circleIntersection(joint, joint1, joint2, link1, link2)
    syms x y;
    circle1 = (x - joint1.x)^2 + (y - joint1.y)^2 == link1.length^2
    circle2 = (x - joint2.x)^2 + (y - joint2.y)^2 == link2.length^2
end
