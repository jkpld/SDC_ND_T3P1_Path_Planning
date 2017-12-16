classdef TrajectoryGeneration
    properties
        % Cost weights
        
        kj = 1;
        kt = 10;
        ks = 100;
        ksd = 400;
        kd = 20000;
        
        
        k_lat = 0.1;
        k_lon = 1;
        
        % Safty_Margin added to the car sizes when determining if there is
        % a collision or not.
        Safty_Margin = 1*[0.5, 0.5];
        
        % Parameters for constant distance and constant time law.
        D0 = 1; % [m]
        tau = 1; % [sec]
        
        % Width of the lanes.
        lane_width = 4; % [m]
        
        % Maximum values the car is not able to be above
        MAX_SPEED = 50 * 0.44704; % [m/s]
        MAX_ACCEL = 10; % [m/s/s]
        MAX_JERK = 10; % [m/s/s/s]
        
        MIN_D = 0;
        MAX_D = 12;
        
        Time_Horizon = 3;
        Reactive_Layer_Time_Horizon = 1;
    end
    properties (Hidden)
        T = 1:1:3; % Time horizons to search for a new trajectory
        ds = -5:1:5; % offset s distance to search for a new trajectory
        dsd = -2:2:2; % offset speed values to search for new trajectory (velocity keeping)
        d = -1:1;%-2.25:4.5/8:2.25; % offset d distances to search for a new trajectory
    end
        
    methods
        function obj = TrajectoryGeneration()
        end       
        
        function [collide, ego, dt] = reactive_layer(obj, ego, cars, Q)
%             start = tic;
%             send(Q,' ---------  In Reactive layer!  ------------')
%             % Construct the car trajectories for collision detection.
%             t = (0:0.1:obj.Reactive_Layer_Time_Horizon).';
%             Nt = numel(t);
%             Nc = numel(cars);
%             car_traj = zeros(Nt,2,Nc);
%             for car_idx = Nc:-1:1
%                 t0 = cars(car_idx).t0;
%                 [x,y] = cars(car_idx).location(t+t0);
%                 car_traj(:,1,car_idx) = x;
%                 car_traj(:,2,car_idx) = y;
%             end
%             
%             % Get the minimum distance at which there cannot be a
%             % collision
%             min_collision_free_dist = ([cars.Length]/2).^2 +([cars.Width]/2).^2 + (ego.Length/2)^2 + (ego.Width/2)^2;
%             
%             % Detect if there will be a collision
%             collide = detect_collision(obj, ego, cars, car_traj, t, min_collision_free_dist);
% %             send(Q, collide)
%             if collide
                % If there a collision was detected, then search for a new
                % trajectory to try and avoid it
                mean_speed = 0;
                for i = 1:numel(cars)
                    mean_speed = mean_speed + cars(i).state.x(2);
                end
                mean_speed = mean_speed/numel(cars);
                activeModes(1) = ActiveMode("velocity_keeping", 21);
%                 activeModes(2) = ActiveMode("stopping", min_speed*1);
%                 activeModes = struct('name',"velocity_keeping",'data',min_speed);
%                 activeModes(2).name = "stopping";
%                 activeModes(2).data = min_speed*1;
                d_search = -6:2:6;
                
                sd_search = [(mean_speed - 21) + (-4:2:4), -2,0];
                
                obj.T = 1:4;
                [ego, collide] = generate(obj, Q, ego, cars, activeModes, 2, d_search, sd_search);
                obj.T = 1:1:3;
%             end
              dt = 0;
%             dt = toc(start);
        end
        
        function [ego, collide] = generate(obj, Q, ego, cars, activeModes, goal_lane, lateral_search_values, sd_vals)
            state0 = ego.state;
            tic
            
            % create valid lateral trajectories
            if nargin < 6
                goal_lane = ceil(state0.y(1) / obj.lane_width);
            end
            if nargin < 7
                lateral_search_values = obj.d;
            end
            if nargin < 8
                sd_vals = obj.dsd;
            end
%             send(Q, obj.d)
%             send(Q, lateral_search_values)
%             send(Q, (goal_lane - 0.5) * obj.lane_width + lateral_search_values)
            [trajs_d, costs_d, is_valid_d] = lateral(obj, state0, goal_lane, lateral_search_values,Q);
            trajs_d = trajs_d(is_valid_d);
            costs_d = costs_d(is_valid_d);
            Nd = numel(costs_d);
            if Nd == 0
                disp("no valid lateral trajs")
                collide = true;
                return
            end
            % Create valid longitudinal trajectories for each active mode
            for i = numel(activeModes):-1:1
                switch activeModes(i).name
                    case "following"
                        [trajs_s_i, costs_s_i, is_valid_s] = following(obj, state0, activeModes(i).data.state,Q);
                        trajs_s{i} = trajs_s_i(is_valid_s);
                        costs_s{i} = costs_s_i(is_valid_s);
                    case "merging"
                        [trajs_s_i, costs_s_i, is_valid_s] = merging(obj, state0, activeModes(i).data(1), activeModes(i).data(2),Q);
                        trajs_s{i} = trajs_s_i(is_valid_s);
                        costs_s{i} = costs_s_i(is_valid_s);
                    case "stopping"
                        [trajs_s_i, costs_s_i, is_valid_s] = stopping(obj, state0, activeModes(i).data,Q);
                        trajs_s{i} = trajs_s_i(is_valid_s);
                        costs_s{i} = costs_s_i(is_valid_s);
                    case "velocity_keeping"
                        [trajs_s_i, costs_s_i, is_valid_s] = velocity_keeping(obj, state0, activeModes(i).data, sd_vals, Q);
                        trajs_s{i} = trajs_s_i(is_valid_s);
                        costs_s{i} = costs_s_i(is_valid_s);
                end
            end
            trajs_s = cat(1,trajs_s{:});
            costs_s = cat(1,costs_s{:});
            
            Ns = numel(costs_s);
            if Ns == 0
                disp("no valid lon trajs")
                collide = true;
                return
            end

            % - Construct the cost for each possible combination of
            % d_trajectory with s_trajectory.
            % - Iterate through the possible combinations in the order of
            % increasing cost (starting with the lowest cost)
            % - Check to see if the current trajectory combination collides
            % with any of the cars. If it does, then move to the next
            % higher cost trajectory combination. If it does not, then
            % return ego with the new trajectory.
            
            cost = obj.k_lat*costs_d(:) + obj.k_lon*costs_s(:)';
            [~, idx] = sort(cost(:)); % idx is the linear index into the matrix d_traj x s_traj
            
            collide = false;
            % Only need to check for collisions of there are other cars.
            if nargin <5 || isempty(cars)
                
                % Set the trajectory to be the lowest cost trajectory and
                % give it to ego
                [d_idx,s_idx] = ind2sub([Nd, Ns], idx(1));
                ego.trajectory = [trajs_s(s_idx), trajs_d(d_idx)];
            else
                
                % Construct the car trajectories for collision detection.
                t = (0:0.2:obj.Time_Horizon).';
                Nt = numel(t);
                Nc = numel(cars);
                car_traj = zeros(Nt,2,Nc);
                for car_idx = Nc:-1:1
                    [x,y] = cars(car_idx).location(t);
                    car_traj(:,1,car_idx) = x;
                    car_traj(:,2,car_idx) = y;
                end
                
                % Get the minimum distance at which there cannot be a
                % collision
                min_collision_free_dist = sqrt(([cars.Length]/2+obj.Safty_Margin(1)).^2 +([cars.Width]/2+obj.Safty_Margin(2)).^2) + sqrt((ego.Length/2+obj.Safty_Margin(1))^2 + (ego.Width/2+obj.Safty_Margin(2))^2);
            
                collide = false;
                % start checking the trajectories for collisions, starting with
                % the lowest cost trajectory and then increasing

                for i = 1:Ns*Nd

                    % Get the index of the d and s trajectory
                    [d_idx,s_idx] = ind2sub([Nd, Ns], idx(i));

                    % Set ego to have the trajectory
                    ego.trajectory = [trajs_s(s_idx), trajs_d(d_idx)];

                    % Check for collision                    
                    collide = detect_collision(obj, ego, cars, car_traj, t, min_collision_free_dist, Q);

                    % If we did not collide with anything, then we are done,
                    % and ego already has the new trajectory;
                    if ~collide
                        break;
                    end
                end
                
%                 d_i = ego.trajectory(2).evaluate(t,0);
%                 send(Q,sprintf('Going off road: %d', any((d_i < obj.MIN_D) | (d_i > obj.MAX_D))))
                
                if collide
                    send(Q,'No valid path found!!!')
                    ego.state = state0;
                    ego.t0 = 0;
%                     [collide, ego, dt] = reactive_layer(obj, ego, cars, Q);
                end
            end
            
%             send(Q,toc);
        end
        
        function collide = detect_collision(obj, ego, cars, car_traj, t, min_collision_free_dist, Q)
           
            Nt = numel(t);
            Nc = numel(cars);
            
            % Get trajectory of ego
            s_i = ego.trajectory(1).evaluate(t,0);
            d_i = ego.trajectory(2).evaluate(t,0);
            
            % Compute the distance to each car as a function of time,
            % and normalize the minimum distance between ego and the
            % other cars for which there cannot be a collision. We thus
            % only need to check the distances that are less than 1, to
            % see if there is a collision or not.
            D2 = permute(sum((car_traj - [s_i, d_i]).^2,2), [1,3,2]) ./ min_collision_free_dist.^2;
            check_idx = find(D2 <= 1); % Find all points with a possible collision, these are linear indices

            if numel(check_idx) > 0
                % Order the points by increasing distance. This will
                % hopefully reduce the number of collision checks are
                % necessary if there is indeed a collision.
                [~,search_order] = sort(D2(check_idx));
                check_idx = check_idx(search_order);

                for ti_ci = 1:numel(check_idx)
                    % Get the time and car index for the current check
                    % point.
                    [ti, ci] = ind2sub([Nt,Nc], check_idx(ti_ci));

                    % Get the bounding boxes of the cars
                    car_bbox = cars(ci).bounding_box(t(ti), obj.Safty_Margin);
                    ego_bbox = ego.bounding_box(t(ti), obj.Safty_Margin);
                    car_bbox = car_bbox + car_traj(ti,:,ci);
                    ego_bbox = ego_bbox + [s_i(ti), d_i(ti)];

                    % Determine if the cars collide using SAT.
                    collide = RectangleCollision(ego_bbox, car_bbox);
                    if collide
                        break;
                    end
                end
            else
                collide = false;
            end
                    
        end
        
        function [trajs, costs, is_valid] = lateral(obj, state0, goal_lane, lateral_search_values,Q)
            
            % Starting state
            d0 = state0.y;
            
            % Compute the current lane and the goal lane
%             lane0 = ceil(d0(1) / obj.lane_width);
%             lane1 = lane0 + lane_offset;
            d_goal = (goal_lane - 0.5) * obj.lane_width; % The goal lane position.
            
            % Grid of search values.
            d = lateral_search_values + d_goal;
            
            d((d<obj.MIN_D) | (d>obj.MAX_D)) = [];
            if isempty(d)
                trajs = Trajectory.empty();
                costs = 9999999;
                is_valid = false;
                return;
            end
            
            [d,T] = meshgrid(d, obj.T);  %#ok<*PROPLC>
            d = d(:);
            T = T(:);
            
            is_valid = true(numel(T), 1);
            costs = zeros(numel(T), 1);
            coefs = zeros(numel(T), 6);
            trajs(numel(T),1) = Trajectory();
            
            for i = 1:numel(T)
                d1 = [d(i), 0, 0];
                  
                coefs(i,:) = JMT(obj, d0, d1, T(i));
                costs(i) = ComputeCost(obj, coefs(i,:), T(i), d1 - [d_goal, 0, 0], [], "lateral");
                is_valid(i) = ComputeValidity(obj, coefs(i,:), T(i), true);
                trajs(i) = Trajectory(coefs(i,:),d1,T(i));
            end
                        
%             coefs = coefs(is_valid,:);
%             costs = costs(is_valid);
%             T = T(is_valid);
        end
        
        function [trajs, costs, is_valid] = following(obj, state0, slv,Q)
            % FOLLOWING Compute trajectoy for following a car, slv, with a
            % starting position s0.
            %
            % [best_coefs, cost] = following(obj, s0, slv)
            
            s0 = state0.x;
            slv = slv.x;
            
            % slv : state of leading vehicle (lv) [x_lv, v_lv, a_lv];
            
            % Create the trajectory of the leading vehicle while including
            % the constant time gap law
            % s_target(t) = s_lv(t) - [D_0 + tau * s_lv_dot(t)];
            
            % slv(t) = slv_1 + slv_2*t + 0.5*slv_3*t^2
            slv = [0.5*slv(3), slv(2), slv(1)]; % slv = polyint(polyint(slv(3),slv(2)),slv(1)); 
            slv_d = [0, polyder(slv)];
            st = slv - ([0, 0, obj.D0] + obj.tau * slv_d);

            [trajs, costs, is_valid] = following_merging_stoping(obj, s0, st, "following",Q);
        end
        
        function [trajs, costs, is_valid] = merging(obj, state0, sa, sb,Q)
            % MERGING Compute trajectory for merging between two cars, sa
            % and sb, with a starting position s0.
            %
            % [best_coefs, cost] = merging(obj, s0, sa, sb)
            
            s0 = state0.x;
            sa = sa.state.x;
            sb = sb.state.x;
            
            % merge between two vehicles given by sa and sb
            % sa = [xa, va, aa];
            % sb = [xb, vb, ab];
            st = (sa + sb)/2;
            st = [0.5*st(3), st(2), st(1)]; % st(t) = x + v*t + 0.5*a*t^2
            [trajs, costs, is_valid] = following_merging_stoping(obj, s0, st, "merging",Q);
        end
        
        function [trajs, costs, is_valid] = stopping(obj, state0, s_end,Q)
            % STOPPING Compute trajectory for stopping at location s_end
            % with starting position s0.
            %
            % [best_coefs, cost] = stopping(obj, s0, s_end)
            
            s0 = state0.x;
            
            % stop at point s_end
            st = [0, 0, s_end];
            [trajs, costs, is_valid] = following_merging_stoping(obj, s0, st, "stopping",Q);
        end
        
        function [trajs, costs, is_valid] = velocity_keeping(obj, state0, s_d, sd_vals, Q)
            
            s0 = state0.x;
            
            dsd = s_d + sd_vals;
            dsd(dsd>obj.MAX_SPEED) = [];
            [dsd,T] = meshgrid(dsd , obj.T);  %#ok<*PROPLC>
            dsd = dsd(:);
            T = T(:);
            
            is_valid = true(numel(T), 1);
            costs = zeros(numel(T), 1);
            coefs = zeros(numel(T), 5);
            trajs(numel(T),1) = Trajectory();
            
            for i = 1:numel(T)
                s1 = [0, dsd(i), 0];
                  
                coefs(i,:) = JMT_vel_keep(obj, s0, s1, T(i));
                % Update what the position should be at time T.
                s1(1) = polyval(coefs(i,:),T(i));
                costs(i) = ComputeCost(obj, coefs(i,:), T(i), s1, s_d, "velocity_keeping");
                is_valid(i) = ComputeValidity(obj, coefs(i,:), T(i));
                trajs(i) = Trajectory(coefs(i,:),s1,T(i));
            end
            
%             coefs = coefs(is_valid,:);
%             costs = costs(is_valid);
%             T = T(is_valid);
        end
    end
    
    methods (Access = private)
        
        function coefs = JMT(~,s0, s1, t)
            
            % Solve for the position trajectory
            t = cumprod(t*ones(1,5));
            
            A = [  t(3),    t(4),    t(5);
                 3*t(2),  4*t(3),  5*t(4);
                 6*t(1), 12*t(2), 20*t(3)];
             
            b = [s1(1) - (s0(1) + s0(2)*t(1) + 0.5*s0(3)*t(2));
                 s1(2) - (s0(2) + s0(3)*t(1));
                 s1(3) - s0(3)];
             
            c = A\b;
            coefs = [c(3), c(2), c(1), 0.5*s0(3), s0([2,1])];
        end
        
        function coefs = JMT_vel_keep(~,s0, s1, t)
            
            % Solve for the velocity trajectory
            t = cumprod(t*ones(1,3));
            
            A = [  t(2),    t(3);
                 2*t(1),  3*t(2)];
             
            b = [s1(2) - (s0(2) + s0(3)*t(1));
                 s1(3) - s0(3)];
             
            c = A\b;
            coefs = [c(2), c(1), s0(3), s0(2)];
            
            % Now integrate to create the position trajectory with a
            % starting location of s0(1)
            coefs = polyint(coefs, s0(1));
        end
        
        function [trajs, costs, is_valid] = following_merging_stoping(obj, s0, s_target, type, Q)
            % FOLLOWING_MERGING_STOPPING Compute trajectory for the three
            % strategies, following, merging, and stoping.
            %
            % s0 should be the starting state [x,v,a].
            % s_target should be the equation of motion polynomial 
            % [0.5*a, v, x], NOT the state [x,v,a].
            
            st_d = polyder(s_target);
            st_dd = polyder(st_d);
            
            [ds,T] = meshgrid(obj.ds, obj.T);  %#ok<*PROPLC>
            ds = ds(:);
            T = T(:);
            
            is_valid = true(numel(T), 1);
            costs = zeros(numel(T), 1);
            coefs = zeros(numel(T), 6);
            trajs(numel(T),1) = Trajectory();
            
            for i = 1:numel(T)
                s1 = [polyval(s_target, T(i)) + ds(i), ...
                      polyval(st_d, T(i)), ...
                      polyval(st_dd, T(i))];
                  
                coefs(i,:) = JMT(obj, s0, s1, T(i));
                costs(i) = ComputeCost(obj, coefs(i,:), T(i), s1, s_target, type);
                is_valid(i) = ComputeValidity(obj, coefs(i,:), T(i), Q);
                trajs(i) = Trajectory(coefs(i,:),s1,T(i));
            end
            
%             coefs = coefs(is_valid,:);
%             costs = costs(is_valid);
%             T = T(is_valid);
        end
        
        function cost = following_merging_stopping_cost(obj, T, s1, target)
            % longitudinal position difference cost
            cost = (s1(1) - polyval(target,T))^2;% + 100*(1-s1(2)/obj.MAX_SPEED).^2;% + 500/((s1(2)/10)^2+0.001);
        end
        
        function cost = velocity_keeping_cost(obj, s1, target_v)
            % velocity difference cost
            cost = (s1(2) - target_v)^2;
        end
        
        function cost = lateral_cost(obj, d1)
            % lateral position difference cost 
            % Note! d1 should be relative to the current lane center.
            cost = (mod(d1(1),obj.lane_width))^2;
%             cost = d1(1)^2;
        end
        
        function cost = ComputeCost(obj, coefs, T, s1, target, type)
            % Jerk
            p3 = polyder(polyder(polyder(coefs)));
            
            % Jerk cost : int(p3^2,t0,t1)
            
            Cj = obj.kj * diff(polyval(polyint(conv(p3,p3)),[0,T]));
            
            % Time cost
            Ct = obj.kt * T;
            
            switch type
                case {"following", "merging", "stopping"}
                    Cd = obj.ks * following_merging_stopping_cost(obj, T, s1, target);
                case "velocity_keeping"
                    Cd = obj.ksd * velocity_keeping_cost(obj, s1, target);
                case "lateral"
                    Cd = obj.kd * lateral_cost(obj, s1);
            end
            
            cost = Cj + Ct + Cd;
            
        end
        
        function is_valid = ComputeValidity(obj, coefs, T, Q)

            is_valid = true(numel(T),1);
            max_vals = [obj.MAX_SPEED, obj.MAX_ACCEL, obj.MAX_JERK];

            for i = 1:numel(T)
                p = polyder(coefs(i,:));
                
                for j = 1:3
                    pp = polyder(p);
                    r = roots(pp);
                    r(r<0 | r>T(i)) = [];

                    v = abs(polyval(p, [r; 0; T(i)]));
                    if any(v>max_vals(j))
                        is_valid(i) = false;
                        break;
                    end
                    p = pp;
                end
            end
        end
    end
end

function pd = polyder(p)
n = numel(p);
if n == 1
    pd = 0;
else
    pd = p(1:n-1).*(n-1:-1:1);
end
end
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    