classdef CarSim < handle
    properties
        fig
        ax
        cars
        ego
        ego_loc
        
        road
        
        egobbox
        egobboxSM
        egoc
        ego_traj
        
        
        carbbox
        carbboxSM
        carc
        
        t
        dt = 0.02;
        
        message
        egoInfo_s
        egoInfo_a
        egoInfo_j
        fps
        lane_nums
        current_time
        aM_info
        
        history_x = []
        history_y = []
        history_size = 15;
        
        FPSclock = -1;
        FPS_hist = [];
        counter = 1;
       
        
        Safty_Margin = [0.5,0.5];
        min_collision_free_dist
        
        
        frames
        frame_period = nan;
        f_counter = 1;
    end
    
    
    methods
        function obj = CarSim(cars, ego)
            cars.Length
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
            cars.Length
            obj.carc = line(nan,nan,'Marker','o','MarkerFaceColor','r','LineStyle','none');
            obj.carbbox = line(nan,nan,'color','r');
            obj.carbboxSM = line(nan,nan,'color',[0.3,0,0],'LineStyle','--');
            
            % Initialize lines for plotting ego
            obj.ego = ego;
            obj.ego_loc = [ego.state.x(1), ego.state.y(1), 0];
            obj.egobbox = line(nan,nan,'color','b');
            obj.egobboxSM = line(nan,nan,'color',[0,0,0.3], 'LineStyle','--');
            obj.egoc = line(nan,nan,'Marker','o','MarkerFaceColor','b');
            obj.ego_traj = line(nan,nan,'color',[0,0.8,0]);
            sqrt(([cars.Length]/2).^2 +([cars.Width]/2).^2)
            obj.min_collision_free_dist = sqrt(([cars.Length]/2).^2 +([cars.Width]/2).^2) + sqrt((ego.Length/2)^2 + (ego.Width/2)^2);
            
            % Create a text box for displaying messages
            obj.message = text(0,0,'','FontSize',20);
            obj.fps = text(0,0,'','FontSize',14, 'VerticalAlignment','bottom');
            obj.egoInfo_s = text(0,0,'','FontSize',10, 'VerticalAlignment','middle','HorizontalAlignment','right');
            obj.egoInfo_a = text(0,0,'','FontSize',10, 'VerticalAlignment','middle','HorizontalAlignment','right');
            obj.egoInfo_j = text(0,0,'','FontSize',10, 'VerticalAlignment','middle','HorizontalAlignment','right');
            
            obj.aM_info = text(0,0,'','FontSize',10, 'VerticalAlignment','baseline','HorizontalAlignment','left');
            
            for i = num_lanes:-1:1
                lane_ns(i) = text(0,(i-0.5)*lane_width, sprintf('%d',i), 'FontSize', 16,'VerticalAlignment','middle','HorizontalAlignment','right');
            end
            obj.lane_nums = lane_ns;
            % Initilize the simulation time
            obj.t = 0;
            
            obj.current_time = text(0,(num_lanes-0.3)*lane_width, '', 'FontSize',12,'VerticalAlignment','middle','HorizontalAlignment','right');
            
            % Make the figure dark theme.
            setTheme(gcf,'dark')
        end
        
        function [collide, ego_pose, ego_path_xy, cars, other_paths] = advance(obj, ego_path_xy, other_paths, SM)

            if obj.FPSclock == -1
                obj.FPSclock = tic;
            end

            try
                delete(findall(obj.ax,'Tag','OtherPaths'))
            catch
            end
            
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
                if isequal(obj.ego_loc(1:2), ego_path_xy(idx,:))
                    idx = idx+1;
                end
                % Pop for the first element off the xy list.
                x = ego_path_xy(idx,1);
                y = ego_path_xy(idx,2);
                ego_path_xy(1:idx,:) = [];
                
                for i = 1:numel(other_paths)
                    other_paths{i}(1:min(idx,size(other_paths{i},1)),:) = [];
                end
                
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

            % Update ego location marker and bounding box
            obj.egoc.XData = x;
            obj.egoc.YData = y;

            bbox = obj.ego.bounding_box(obj.t,[0,0],th) + [x,y];
            bboxSM = obj.ego.bounding_box(obj.t,obj.Safty_Margin,th) + [x,y];
            obj.egobbox.XData = bbox([1:end,1],1);
            obj.egobbox.YData = bbox([1:end,1],2);
            obj.egobboxSM.XData = bboxSM([1:end,1],1);
            obj.egobboxSM.YData = bboxSM([1:end,1],2);
            egox = x;
            egoy = y;
            
            if ~isempty(SM) 
                aM = SM.activeModes;
                names = [aM.name];

                obj.aM_info.String = strrep(names.join(", "),"_", " ");
                obj.aM_info.Position(1:2) = [egox - obj.ego.Length/2, y + obj.ego.Width/2];
            else
            end
            
            % Update axis x-lims
            obj.ax.XLim = x + [0, diff(obj.ax.YLim)*1600/300] -20;
            
            % Update ego information
            [s,a,j] = obj.compute_speed_accel_jerk();
            obj.update_ego_info(x,y,s,a,j)
            ego_pose = struct('x',x,'y',y,'speed',s,'yaw',th);
            
            % Update ego trajectory
            if ~isempty(ego_path_xy)
                obj.ego_traj.XData = ego_path_xy(:,1);
                obj.ego_traj.YData = ego_path_xy(:,2);
            end
            
            % Plot any other trajectories
%             other_paths
            for i = 1:numel(other_paths)
                line(other_paths{i}(:,1),other_paths{i}(:,2), 'color',[0,0.4,0],'Parent',obj.ax,'Tag','OtherPaths')
            end
            
            
            % Update other cars locations and bounding boxes, and check for
            % collisions
            collide = false;
            bboxline = nan(numel(obj.cars)*6 - 1,2);
            bboxlineSM = nan(numel(obj.cars)*6 - 1,2);
            carc_x = nan(1,numel(obj.cars));
            carc_y = nan(1,numel(obj.cars));
            
            for j = 1:numel(obj.cars)
    
                [x,y] = obj.cars(j).location(obj.t);
                bbox1 = obj.cars(j).bounding_box(obj.t, [0,0]) + [x,y];
                bbox1SM = obj.cars(j).bounding_box(obj.t, obj.Safty_Margin) + [x,y];

                
                if sqrt(sum(([x,y]-[egox,egoy]).^2)) <= obj.min_collision_free_dist(j)
                    collide = collide || RectangleCollision( bbox, bbox1);
                end

                % only draw car if it within the axis limits.
                x_dist = (x+[1,-1]*(obj.cars(j).Length/2 + obj.Safty_Margin(1)));
                if  x_dist(1) > obj.ax.XLim(1) && x_dist(2) < obj.ax.XLim(2)
                    bboxline((1:5) + 6*(j-1),:) = bbox1([1:end,1],:);
                    bboxlineSM((1:5) + 6*(j-1),:) = bbox1SM([1:end,1],:);

                    carc_x(j) = x;
                    carc_y(j) = y;
                end
            end
            
            obj.carc.XData = carc_x;
            obj.carc.YData = carc_y;
            obj.carbbox.XData = bboxline(:,1);
            obj.carbbox.YData = bboxline(:,2);
            obj.carbboxSM.XData = bboxlineSM(:,1);
            obj.carbboxSM.YData = bboxlineSM(:,2);
            
            % If there is a collision then display a message.
            if collide
                obj.message.Position(1) = mean(obj.ax.XLim);
                obj.message.Position(2) = obj.ax.YLim(2) - 0.14*diff(obj.ax.YLim);
                obj.message.String = 'Collide!';
            else
                obj.message.String = '';
            end

            % Update all car states to be relative to current time.
            update_car_states(obj);
            cars = get_cars(obj); % Get the cars to output.
            
            
            % Update lane number positions
            for i = 1:numel(obj.lane_nums)
                obj.lane_nums(i).Position(1) = obj.ax.XLim(2);
            end
            
            obj.current_time.Position(1) = obj.ax.XLim(2);
            obj.current_time.String = sprintf('%0.2f', obj.t);
            
            while toc(obj.FPSclock) < 0.015
            end
            
            FPS = 1/toc(obj.FPSclock);
            obj.FPS_hist(mod(ceil(obj.t/obj.dt), obj.history_size)+1) = FPS;
            obj.fps.Position(1:2) = [obj.ax.XLim(1), obj.ax.YLim(2) - 0.14*diff(obj.ax.YLim)];
%             obj.fps.String = sprintf('FPS = %0.0f',mean(obj.FPS_hist));
            obj.fps.String = '';
            
            obj.FPSclock = tic;
%             drawnow;
            if mean(obj.FPS_hist)>50
                drawnow;
            end
            
            if isfinite(obj.frame_period)
                if mod(obj.counter,obj.frame_period) == 0
%                 if mod(obj.t,obj.frame_period) < 0.01
fprintf('CAPTURING FRAME -----------------------------------\n');
                    drawnow;
                    f = getframe(obj.fig);
                    obj.frames{obj.f_counter} = frame2im(f);
                    obj.f_counter = obj.f_counter + 1;
                end
            end
            
            % Update the time for the next call
            obj.t = obj.t + obj.dt;
            obj.counter = obj.counter + 1;
            
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
            if numel(obj.history_x) == obj.history_size
                obj.history_x = circshift(obj.history_x,-1);
                obj.history_y = circshift(obj.history_y,-1);
                obj.history_x(end) = x;
                obj.history_y(end) = y;
            else
                obj.history_x(end+1) = x;
                obj.history_y(end+1) = y;
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
                speed = median(sqrt(vx.^2 + vy.^2));
            end
            if N > 2
                a_x = diff(vx)/obj.dt;
                ay = diff(vy)/obj.dt;
                accel = median(sqrt(a_x.^2 + ay.^2));
            end
            if N > 3
                jx = diff(a_x)/obj.dt;
                jy = diff(ay)/obj.dt;
                jerk = median(sqrt(jx.^2 + jy.^2));
            end
        end
        
        function record(obj,total_count, hz)
            total_time = total_count*obj.dt;
            period = floor(1/(hz*obj.dt))*obj.dt;
            fprintf('Using period of %0.3f\n', period)
            num_frames = floor(total_time / period);
            obj.frames = cell(1,num_frames);
            obj.frame_period = round(period/obj.dt);
            obj.f_counter = 1;
            
        end
        
        function reset(obj)
            % Reset the simulations (car states and history)
            obj.counter = 1;
            obj.t = 0;
            obj.update_car_states();
            obj.history_x = [];
            obj.history_y = [];
            obj.ego_loc = [obj.ego.state.x(1), obj.ego.state.y(1), 0];
            obj.FPS_hist = [];
            
            obj.frames = cell(1,0);
            obj.frame_period = nan;
        end
    end
end