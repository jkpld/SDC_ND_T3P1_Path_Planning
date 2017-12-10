classdef CarSim < handle
    properties
        fig
        ax
        cars
        ego
        ego_loc
        
        road
        
        egobbox
        egoc
        ego_traj
        
        
        carbbox
        carc
        
        t
        dt = 0.02;
        
        message
        egoInfo_s
        egoInfo_a
        egoInfo_j
        
        history_x = []
        history_y = []
        history_size = 25;
    end
    
    
    methods
        function obj = CarSim(cars, ego)
            
            try close(findobj('Type','Figure','Name','CarSim')), catch, end
            obj.fig = figure('Position',[300 630 1600 300],'Name','CarSim');
            obj.ax = axes('Parent',obj.fig,'Units','norm','Position',[0,0.15,1,0.85]);
            
            % Create road
            num_lanes = 3;
            road_x = [-1000,2000];
            lane_width = 4;
            
            obj.road.yellow(2) = line(road_x,num_lanes*lane_width + [0.15,0.15],'color','y','linewidth',2,'Parent',obj.ax);
            obj.road.yellow(1) = line(road_x,num_lanes*lane_width - [0.1,0.1],'color','y','linewidth',2,'Parent',obj.ax);
            
            for i = (num_lanes-1):-1:1
                obj.road.lane(i) = line(road_x,lane_width*i*[1 1], 'color','w','LineStyle','--','LineWidth',2,'Parent',obj.ax);
            end
            
            obj.road.center = line(road_x, [0 0], 'color','w','LineStyle','-','LineWidth',2,'Parent',obj.ax);
            
            % Set the y-axis limits
            obj.ax.YLim = [-0.5,num_lanes*lane_width+0.5];
            
            % Add cars and initialize lines to plot them
            if nargin == 0
                car(5) = Vehicle();
                car(1).state = State(0,[0,10,0],[2,0,0]);
                car(2).state = State(0,[12,3,0],[6,0,0]);
                car(3).state = State(0,[0,17,0],[10,0,0]);
                car(4).state = State(0,[15,17,0],[10,0,0]);

                car(5).Length = 7;
                car(5).state = State(0,[17,10,0.2],[2,0,0]);

                obj.cars = car;
            else
                obj.cars = cars;
            end
            
            for i = numel(obj.cars):-1:1
                carbbox(i) = line(nan,nan,'color','r');
                carc(i) = line(nan,nan,'Marker','o','MarkerFaceColor','r');
            end
            obj.carbbox = carbbox;
            obj.carc = carc;
            
            % Initialize lines for plotting ego
            obj.ego = ego;
            obj.ego_loc = [ego.state.x(1), ego.state.y(1), 0];
            obj.egobbox = line(nan,nan,'color','b');
            obj.egoc = line(nan,nan,'Marker','o','MarkerFaceColor','b');
            obj.ego_traj = line(nan,nan);
            
            % Create a text box for displaying messages
            obj.message = text(0,0,'','FontSize',20);
            obj.egoInfo_s = text(0,0,'','FontSize',10, 'VerticalAlignment','middle','HorizontalAlignment','right');
            obj.egoInfo_a = text(0,0,'','FontSize',10, 'VerticalAlignment','middle','HorizontalAlignment','right');
            obj.egoInfo_j = text(0,0,'','FontSize',10, 'VerticalAlignment','middle','HorizontalAlignment','right');
            
            % Initilize the simulation time
            obj.t = 0;
            
            % Make the figure dark theme.
            setTheme(gcf,'dark')
        end
        
        function [collide, ego_pose, ego_path_xy, cars] = advance(obj, ego_path_xy)

            if isempty(ego_path_xy)
                % If there are no points in the locations list, the use the
                % last location.
                x = obj.ego_loc(1);
                y = obj.ego_loc(2);
                th = obj.ego_loc(3);
            else
                % When a new set of points it input, then the first few
                % points could be out of data, thus, use the point that is
                % closest to the last position of ego
                d = sum((ego_path_xy - obj.ego_loc(1:2)).^2,2);
                [~,idx] = min(d);
                % Pop for the first element off the xy list.
                x = ego_path_xy(idx,1);
                y = ego_path_xy(idx,2);
                ego_path_xy(1:idx,:) = [];
                
                % Compute the angle ego. (This should be approximately how
                % the c++ simulator does it since it does not have any
                % other information.)
                th = atan2(y-obj.history_y(end), x-obj.history_x(end));
                
                % Update the ego location.
                obj.ego_loc(1) = x;
                obj.ego_loc(2) = y;
                obj.ego_loc(3) = th;
            end
            
            % Add the new location to the history for computing speed,
            % acceleration, and jerk.
            obj.add_to_history(x,y);
            
%             [x,y] = ego.location(obj.t);
%             [v_x,v_y] = ego.speed(obj.t);
%             [a_x,a_y] = ego.accel(obj.t);
%             [j_x,j_y] = ego.jerk(obj.t);

            % Update ego location marker and bounding box
            obj.egoc.XData = x;
            obj.egoc.YData = y;

            bbox = obj.ego.bounding_box(obj.t,[0.5,0.5],th) + [x,y];
            obj.egobbox.XData = bbox([1:end,1],1);
            obj.egobbox.YData = bbox([1:end,1],2);
            
            % Update axis x-lims
            obj.ax.XLim = x + [0, diff(obj.ax.YLim)*1600/300] -20;
            
            % Update ego information
            [s,a,j] = obj.compute_speed_accel_jerk();
            obj.update_ego_info(x,y,s,a,j)
            ego_pose = struct('x',x,'y',y,'speed',s,'yaw',th);
            
%             s = sqrt(v_x^2 + v_y^2);
%             a = sqrt(a_x^2 + a_y^2);
%             j = sqrt(j_x^2 + j_y^2);
            
            
            % Update ego trajectory
%             [x,y] = ego.location(obj.t + (0:0.1:1));
            if ~isempty(ego_path_xy)
                obj.ego_traj.XData = ego_path_xy(:,1);
                obj.ego_traj.YData = ego_path_xy(:,2);
            end
            
            % Update other cars locations and bounding boxes, and check for
            % collisions
            collide = false;
            for j = 1:numel(obj.cars)
    
                [x,y] = obj.cars(j).location(obj.t);
                bbox1 = obj.cars(j).bounding_box(obj.t, [0.5,0.5]) + [x,y];

                collide = collide || RectangleCollision( bbox, bbox1);

                obj.carbbox(j).XData = bbox1([1:end,1],1);
                obj.carbbox(j).YData = bbox1([1:end,1],2);
                
                obj.carc(j).XData = x;
                obj.carc(j).YData = y;
            end
            
            % If there is a collision then display a message.
            if collide
                obj.message.Position(1) = mean(obj.ax.XLim);
                obj.message.Position(2) = obj.ax.YLim(2) - 0.1*diff(obj.ax.YLim);
                obj.message.String = 'Collide!';
            else
                obj.message.String = '';
            end

            % Update all car states to be relative to current time.
            update_car_states(obj);
            cars = get_cars(obj); % Get the cars to output.
            
            % Update the time for the next call
            obj.t = obj.t + obj.dt;
        end
        
        function update_ego_info(obj,x,y,s,a,j)
            % Update the strings behind ego to show the current speed,
            % acceleration, and jerk
            
            obj.egoInfo_s.Position(1:2) = [x-3, y + 0.7];
            obj.egoInfo_s.String = sprintf('s = %0.2f', s);
            if s >= 50 * 0.44704
                obj.egoInfo_s.Color = 'r';
            else
                obj.egoInfo_s.Color = 0.6*[1 1 1];
            end
            obj.egoInfo_a.Position(1:2) = [x-3, y];
            obj.egoInfo_a.String = sprintf('a = %0.2f', a);
            if abs(a) >= 10
                obj.egoInfo_a.Color = 'r';
            else
                obj.egoInfo_a.Color = 0.6*[1 1 1];
            end
            
            obj.egoInfo_j.Position(1:2) = [x-3, y - 0.7];
            obj.egoInfo_j.String = sprintf('j = %0.2f', j);
            if abs(j) >= 10
                obj.egoInfo_j.Color = 'r';
            else
                obj.egoInfo_j.Color = 0.6*[1 1 1];
            end
        end
        
        function update_car_states(obj)
            % Get the car states, relative to now.
            for j = 1:numel(obj.cars)
                obj.cars(j).state = obj.cars(j).state_at(obj.t);
                obj.cars(j).t0 = obj.t;
            end
        end
        
        function cars = get_cars(obj)
            cars = obj.cars;
            for j = 1:numel(obj.cars)
                cars(j).t0 = 0;
            end
        end
        
        function add_to_history(obj,x,y)
            % Add points to history
            obj.history_x(end+1) = x;
            obj.history_y(end+1) = y;
            if numel(obj.history_x) > obj.history_size
                obj.history_x(1) = [];
                obj.history_y(1) = [];
            end
        end
        
        function [speed, accel, jerk] = compute_speed_accel_jerk(obj)
            % Get the mean speed, acceleration, and jerk from the history
            % of xy points.
            speed = 0;
            accel = 0;
            jerk = 0;
            N = numel(obj.history_x);
            if N > 1
                vx = diff(obj.history_x)/obj.dt;
                vy = diff(obj.history_y)/obj.dt;
                speed = mean(sqrt(vx.^2 + vy.^2));
            end
            if N > 2
                a_x = diff(vx)/obj.dt;
                ay = diff(vy)/obj.dt;
                accel = mean(sqrt(a_x.^2 + ay.^2));
            end
            if N > 3
                jx = diff(a_x)/obj.dt;
                jy = diff(ay)/obj.dt;
                jerk = mean(sqrt(jx.^2 + jy.^2));
            end
        end
        
        function reset(obj, ego)
            % Reset the simulations (car states and history)
            obj.t = 0;
            obj.update_car_states();
            obj.history_x = [];
            obj.history_y = [];
            
            obj.ego = ego;
            obj.ego_loc = [ego.state.x(1), ego.state.y(1), 0];
        end
    end
end