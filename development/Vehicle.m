classdef Vehicle
    properties
        ID(1,1) uint16 
        Length(1,1) double = 4.8
        Width(1,1) double = 1.8
        
        t0(1,1) double
        
        last_collision_check_t(1,1) double
    end
    properties (Dependent)
        state
        trajectory
    end
    properties (Access = private)
        bbox
        update_info
        bbox_vert = [-1,-1; -1, 1; 1, 1; 1, -1];
        
        state_(1,1) State
        trajectory_(1,2) Trajectory
    end
    methods
        function obj = Vehicle(length, width)
            obj.ID = randi(intmax('uint16'),1);
            if nargin > 0
                obj.Length = length;
            end
            if nargin > 1
                obj.Width = width;
            end
            
            obj = set_bbox(obj);
            obj.t0 = 0;
            obj.last_collision_check_t = nan;
        end

        function obj = set.state(obj,state)
            obj.state_ = state;
            [trj_x,trj_y] = obj.state.get_trajectory();
            obj.trajectory_ = [trj_x, trj_y];            
        end
        
        function state = get.state(obj)
            state = obj.state_;
        end
        
        function obj = set.trajectory(obj,trj)
            obj.trajectory_ = trj;
            x = obj.trajectory(1).state_at(0);
            y = obj.trajectory(1).state_at(0);
            obj.state_ = State(0,x,y);
        end
        
        function traj = get.trajectory(obj)
            traj = obj.trajectory_;
        end
        
        function obj = set.Length(obj,length)
            obj.Length = length;
            obj = set_bbox(obj);
        end
        
        function obj = set.Width(obj,width)
            obj.Width = width;
            obj = set_bbox(obj);
        end
        
        function theta = orientation(obj, t)
            % Compute the car orientation from the trajectory at time t
            t = t - obj.t0;
            T = max([obj.trajectory.T]);
            t(t>=T) = T-0.01;
            
            vx = obj.trajectory(1).evaluate(t,1); % s dot
            vy = obj.trajectory(2).evaluate(t,1); % d dot
            
            theta = atan2(vy,vx);
        end
        
        function state = state_at(obj, t)
            state = State(obj.trajectory(1).state_at(t - obj.t0), obj.trajectory(2).state_at(t - obj.t0));
        end
        
        function [traj_x, traj_y] = trajectory_at(obj, t)
            t = t - obj.t0;
            
            if t < obj.trajectory(1).T
                % use first polynomial peice and keep second
            else
                % use only second polynomial peice
            end
        end
        
        function [x,y] = location(obj, t)
            x = obj.trajectory(1).evaluate(t - obj.t0);
            y = obj.trajectory(2).evaluate(t - obj.t0);
        end
        
        function [x,y] = speed(obj, t)
            x = obj.trajectory(1).evaluate(t - obj.t0,1);
            y = obj.trajectory(2).evaluate(t - obj.t0,1);
        end
        
        function [x,y] = accel(obj, t)
            x = obj.trajectory(1).evaluate(t - obj.t0,2);
            y = obj.trajectory(2).evaluate(t - obj.t0,2);
        end
        
        function [x,y] = jerk(obj, t)
            x = obj.trajectory(1).evaluate(t - obj.t0,3);
            y = obj.trajectory(2).evaluate(t - obj.t0,3);
        end
        
        function bbox_t = bounding_box(obj, t, safty_margin)
            % Assume we can get heading from derivative of trajectory,
            % should be correct as long as we do not go normal to the road.
            if numel(t) > 1
                error('Vehicle:bounding_box_scalar','Only scalar time is supported')
            end
            if nargin < 3
                safty_margin = [0 0];
            end

            th = orientation(obj,t);

            ct = cos(th);
            st = sin(th);
            rot = [ct, -st; st, ct];
            
            bbox_t = (rot*(obj.bbox + obj.bbox_vert.*safty_margin).').';
        end
    end
   
    methods (Access = private)
        function obj = set_bbox(obj)
            obj.bbox = obj.bbox_vert .* [obj.Length, obj.Width]./2;
        end
    end
end