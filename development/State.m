classdef State
    properties
        coordinateSystem CoordinateSystems = CoordinateSystems.Frenet
%         t0(1,1) double
        x(1,3) double
        y(1,3) double
    end
    
    methods
        function obj = State(x,y,coord)
            if nargin == 0
                return;
            end
            
%             obj.t0 = t0;
            
            obj.x = x;
            obj.y = y;
            if nargin > 3
                obj.coordinateSystem = coord;
            end
        end
        
%         function obj_t = state_at(obj, t)
%             xn = obj.traj_x.state_at(t-obj.t0);
%             yn = obj.traj_y.state_at(t-obj.t0);
%             
%             obj_t = State(t, xn, yn, obj.coordinateSystem);
%         end
        
        function [traj_x, traj_y] = get_trajectory(obj)
            x_t = [0.5*obj.x(3), obj.x(2), obj.x(1)];
            y_t = [0.5*obj.y(3), obj.y(2), obj.y(1)];
            
            traj_x = Trajectory(x_t);
            traj_y = Trajectory(y_t);
        end
    end
end