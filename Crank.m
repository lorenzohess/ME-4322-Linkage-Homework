classdef Crank < Link
    properties
    end
    methods
        function updatedCoords = updateCoords(obj)
            for joint = obj.joints
                if (~joint.ground)
                    newX = obj.groundJoint.x + obj.length * cos(deg2rad(obj.angle));
                    newY = obj.groundJoint.y + obj.length * sin(deg2rad(obj.angle));
                    joint.setCoords(newX, newY);
                    updatedCoords = [newX, newY];
                end
            end
        end
    end
end
