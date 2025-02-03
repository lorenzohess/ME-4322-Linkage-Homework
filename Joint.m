classdef Joint < handle
    properties (SetAccess = private)
        type = "Revolute"; % default
        links % list of Link
        name
        ground = false; % default
    end
    properties
        x % global x coordinate
        y % global y coordinate
    end
    methods
        function obj = Joint(name, x, y, ground)
            obj.name = name;
            obj.x = x;
            obj.y = y;
            if (ground)
                obj.ground = ground;
            end
        end

        % Getters
        function coords = getCoords(obj)
            coords = [obj.x, obj.y];
        end

        function name = get.name(obj)
            name = obj.name;
        end

        % Setters
        function setCoords(obj, newX, newY)
            obj.x = newX;
            obj.y = newY;
        end

        function set.x(obj, newX)
            obj.x = newX;
        end

        function set.y(obj, newY)
            obj.y = newY;
        end

        function set.links(obj, links)
            obj.links = link;
        end
    end
end
