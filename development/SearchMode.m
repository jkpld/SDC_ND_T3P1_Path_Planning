classdef SearchMode
    properties
        activeModes
        goal_lane
        min_lv_speed = -1
    end
    methods
        function obj = SearchMode(activeModes, goal_lane, min_lv_speed)
            obj.activeModes = activeModes;
            obj.goal_lane = goal_lane;
            if nargin > 2
                obj.min_lv_speed = min_lv_speed;
            end
        end
    end
end