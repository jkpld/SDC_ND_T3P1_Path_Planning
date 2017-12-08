classdef CarSim < handle
    properties
        fig
        ax
        cars
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
        function obj = CarSim(cars)
            
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
%             obj.ego_loc = [ego.state.x(1), ego.state.y(1)];
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
        
        function collide = advance(obj, ego)

%             if isempty(ego_xy)
%                 x = obj.ego_loc(1);
%                 y = obj.ego_loc(2);
%             else
%                 x = ego_xy(1,1);
%                 y = ego_xy(1,2);
%                 ego_xy(1,:) = [];
%                 obj.ego_loc(1) = x;
%                 obj.ego_loc(2) = y;
%             end
            [x,y] = ego.location(obj.t);
%             [v_x,v_y] = ego.speed(obj.t);
%             [a_x,a_y] = ego.accel(obj.t);
%             [j_x,j_y] = ego.jerk(obj.t);

            obj.egoc.XData = x;
            obj.egoc.YData = y;

            bbox = ego.bounding_box(obj.t) + [x,y];
            obj.egobbox.XData = bbox([1:end,1],1);
            obj.egobbox.YData = bbox([1:end,1],2);
            
            obj.ax.XLim = x + [0, diff(obj.ax.YLim)*1600/300] -20;
            obj.add_to_history(x,y);
            [s,a,j] = obj.compute_speed_accel_jerk();
            
%             s = sqrt(v_x^2 + v_y^2);
%             a = sqrt(a_x^2 + a_y^2);
%             j = sqrt(j_x^2 + j_y^2);
            
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
            
            [x,y] = ego.location(obj.t + (0:0.1:1));
%             [x(:),y(:)]
            obj.ego_traj.XData = x;%ego_xy(:,1);
            obj.ego_traj.YData = y;%ego_xy(:,2);
            
            collide = false;
            
            for j = 1:numel(obj.cars)
    
                [x,y] = obj.cars(j).location(obj.t);
                bbox1 = obj.cars(j).bounding_box(obj.t) + [x,y];

                collide = collide || RectangleCollision( bbox, bbox1);

                obj.carbbox(j).XData = bbox1([1:end,1],1);
                obj.carbbox(j).YData = bbox1([1:end,1],2);
                
                obj.carc(j).XData = x;
                obj.carc(j).YData = y;
            end
            
            if collide
                obj.message.Position(1) = mean(obj.ax.XLim);
                obj.message.Position(2) = obj.ax.YLim(2) - 0.1*diff(obj.ax.YLim);
                obj.message.String = 'Collide!';
            else
                obj.message.String = '';
            end

            update_car_states(obj);
            obj.t = obj.t + obj.dt;
        end
        
        function update_car_states(obj)
            for j = 1:numel(obj.cars)
                obj.cars(j).state = obj.cars(j).state_at(obj.t);
                obj.cars(j).t0 = obj.t;
            end
        end
        
        function add_to_history(obj,x,y)
            obj.history_x(end+1) = x;
            obj.history_y(end+1) = y;
            if numel(obj.history_x) > obj.history_size
                obj.history_x(1) = [];
                obj.history_y(1) = [];
            end
        end
        
        function [speed, accel, jerk] = compute_speed_accel_jerk(obj)
            speed = nan;
            accel = nan;
            jerk = nan;
            N = numel(obj.history_x);
            if N > 1
                vx = diff(obj.history_x)/obj.dt;
                vy = diff(obj.history_y)/obj.dt;
                speed = mean(sqrt(vx.^2 + vy.^2));
            end
            if N > 2
                ax = diff(vx)/obj.dt;
                ay = diff(vy)/obj.dt;
                accel = mean(sqrt(ax.^2 + ay.^2));
            end
            if N > 3
                jx = diff(ax)/obj.dt;
                jy = diff(ay)/obj.dt;
                jerk = mean(sqrt(jx.^2 + jy.^2));
            end
        end
        
        function reset(obj)
            obj.t = 0;
            obj.update_car_states();
            obj.history_x = [];
            obj.history_y = [];
        end
    end
end