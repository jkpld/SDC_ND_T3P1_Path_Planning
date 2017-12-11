classdef BehaviorModule
    properties
        Behavior_Horizon
        Consider_Distance
        Lane_Width
        Num_Lanes
        Goal_Speed
        
        ego_Length
        ego_Width
        
        trajGen
    end
    properties
        Goal_Lane = -1;
        Min_Tight_Merge_Offset = 2;
        Max_Tight_Merge_Offset = 20
    end
    methods
        function obj = BehaviorModule(ego, Behavior_Horizon, Consider_Distance, Lane_Width, Num_Lanes, Goal_Speed, trajGen)
            obj.ego_Length = ego.Length;
            obj.ego_Width = ego.Width;
            obj.Behavior_Horizon = Behavior_Horizon;
            obj.Consider_Distance = Consider_Distance;
            obj.Lane_Width = Lane_Width;
            obj.Num_Lanes = Num_Lanes;
            obj.Goal_Speed = Goal_Speed;
            obj.trajGen = trajGen;
        end
        
        function [obj, prop_ego, collide, other_egos] = PlanPath(obj, ego, cars, Q)
            
            % Initialize Goal_Lane, if not yet initialized.
            if obj.Goal_Lane == -1
                obj.Goal_Lane = ceil(ego.state.y(1) / obj.Lane_Width);
            end
            
            % Number of cars
            Nc = numel(cars); 
            
            % Get list of car locations now and at the Behavior_Horizon
            r0 = zeros(Nc,2);
            car_speed = zeros(Nc,2);
            r1 = zeros(Nc,2);
            for i = 1:Nc
                % Location now.
                r0(i,:) = [cars(i).state.x(1), cars(i).state.y(1)];
                s0 = sqrt(cars(i).state.x(2)^2 + cars(i).state.y(2).^2);
                
                % Location in future.
                state1 = cars(i).state_at(obj.Behavior_Horizon);
                r1(i,:) = [state1.x(1), state1.y(1)];
                s1 = sqrt(state1.x(2)^2 + state1.y(2)^2);
                
                car_speed(i) = 0.5*(s0+s1);
                
%                 send(Q,cars(i).ID)
%                 send(Q,cars(i).state)
            end
            l0 = ceil(r0(:,2) / obj.Lane_Width);
            
            % Get ego locations now and at the Behavior_Horizon
            er0 = [ego.state.x(1), ego.state.y(1)];
            ego_speed = sqrt(ego.state.x(2)^2 + ego.state.y(2)^2);
            state1 = ego.state_at(obj.Behavior_Horizon);
            er1 = [state1.x(1), state1.y(1)];
            
            % Determine which cars to consider and which to ignore. Cars to
            % consider will be cars that are within Consider_Distance at
            % time 0, or at time Behavior_Horizon
            dr0 = r0 - er0; % Relative distance between cars and ego now.
            dr1 = r1 - er1; % Relative distance between cars and ego at Behavior_Horizon.

            to_consider = any(abs([dr0(:,1), dr1(:,1)]) < obj.Consider_Distance, 2);            
            dr0 = dr0(to_consider,:);
            dr1 = dr1(to_consider,:);
            l0 = l0(to_consider);
            car_speed = car_speed(to_consider);
            cars = cars(to_consider);
            
            Nc = numel(cars); % Number of cars to consider
            send(Q,sprintf('Number of cars : %d', Nc))
            % Get ego's lane, speed, and the set_speed value for
            % "velocity_keeping"
            ego_lane = ceil( ego.state.y(1) / obj.Lane_Width );
            set_speed = min(ego_speed+10, obj.Goal_Speed);
            
            search_modes = SearchMode.empty(); % Initialize vector of search modes
            if (Nc == 0)
                % If there are no cars, then use velocity keeping.    
                activeModes = ActiveMode("velocity_keeping",set_speed);
                search_mode = SearchMode(activeModes, ego_lane);
                search_modes(end+1) = search_mode;
            else
                % There are cars. First create a search mode for staying in
                % the current lane.
                [search_mode, try_switch] = Search_Center(obj, dr0, dr1, l0, car_speed, cars, set_speed, Q);
                search_modes(end+1) = search_mode;
                
%                 for i = 1:numel(search_mode.activeModes)
%                     send(Q, search_mode.activeModes(i))
%                     send(Q, search_mode.activeModes(i).data)
%                     try
%                         send(Q, search_mode.activeModes(i).data.state)
%                     catch
%                     end
%                 end

                if try_switch
                    % Create search modes for other lanes
                    Ncl = 999;
                    if obj.Goal_Lane < obj.Num_Lanes && abs(obj.Goal_Lane+1 - ego_lane) <= 1
                        [search_mode_L, Ncl] = Search_LeftRight(obj, 1, dr0, dr1, l0, car_speed, cars, ego_speed, set_speed, Q);
                    end

                    if Ncl == 0
                        % If there were no cars in the left lane, then we
                        % do not need to search the right lane.
                        send(Q,'Searching left ...............')
                        search_modes(end+1) = search_mode_L;
                    else
                        
                        if obj.Goal_Lane > 1 && abs(obj.Goal_Lane-1 - ego_lane) < 1.1
                            [search_mode_R, Ncr] = Search_LeftRight(obj, -1, dr0, dr1, l0, car_speed, cars, ego_speed, set_speed, Q);
                            if Ncr == 0
                                % If there were no cars int he right lane,
                                % then we do not need to try and use the
                                % left lane.
                                send(Q,'Searching right ...............')
                                search_modes(end+1) = search_mode_R;
                            else
                                if Ncl < 999
                                    send(Q,'Searching left ...............')
                                    search_modes(end+1) = search_mode_L;
                                end
                                send(Q,'Searching right ...............')
                                search_modes(end+1) = search_mode_R;
                            end
                        else
                            if Ncl < 999
                                send(Q,'Searching left ...............')
                                search_modes(end+1) = search_mode_L;
                            end
                        end
                    end
                end
            end
            
            all_collide = true;
            for i = numel(search_modes):-1:1
                [proposed_ego(i), collide] = obj.trajGen.generate(Q, ego, cars, search_modes(i).activeModes, search_modes(i).goal_lane);
                
                if collide
                    cost(i) = inf;
                else
                    cost(i) = ComputeCost(obj, proposed_ego(i), search_modes(i));%99999*collide;
                end
                
                all_collide = collide && all_collide;
            end
            
%             [collide, prop_ego] = obj.trajGen.reactive_layer(ego, cars, Q);

            if all_collide
                [collide, prop_ego] = obj.trajGen.reactive_layer(ego, cars, Q);
                other_egos = proposed_ego;                
                if collide
                    send(Q,'No valid path found for all modes, and reactive mode failed!!!')
                end
            else
            
                [~,idx] = min(cost);
                other_egos = proposed_ego;
                other_egos(idx) = [];
                prop_ego = proposed_ego(idx);
                obj.Goal_Lane = search_modes(idx).goal_lane;
    %             send(Q,obj.Goal_Lane)
                collide = all_collide;
            end
%             if collide
%                 [collide, ego] = obj.trajGen.reactive_layer(ego, cars, Q);
%                 send(Q,'No valid path found for all modes!!!')
%             end
        end
        
        function cost = ComputeCost(obj, ego, search_mode)
            [states_x, states_y] = generate_states(ego, obj.Behavior_Horizon);
            s = sqrt(states_x(2)^2 + states_y(2)^2);
            
            % Higher cost for slower final speed.
            cost = 2*(1 - s/obj.Goal_Speed);
            
            % Higher cost for slower cars in front.
            if search_mode.min_lv_speed ~= -1
                cost = cost + (1-search_mode.min_lv_speed/obj.Goal_Speed);
            end
            
            cost = cost + 0.2*(search_mode.goal_lane - obj.Goal_Lane)^2;
        end
        
        function [search_mode, Nc] = Search_LeftRight(obj, left_right, dloc0, dloc1, car_lane, car_speed, cars, ego_speed, set_speed, Q)
            % Get all cars in the left the lane left of use
            in_lane = obj.Goal_Lane + left_right == car_lane;
            dloc0 = dloc0(in_lane,:);
            dloc1 = dloc1(in_lane,:);
            cars = cars(in_lane);
            
            activeModes = ActiveMode("velocity_keeping", set_speed);
            Nc = numel(cars);
            mlv_s = -1; % minimum leading vehicle speed
            
            if (Nc == 1)
                % Just add "following" and let the cost functions determine
                % if it is good or not. Not the most optimized, but simple.
                if dloc0(1,1) > 0 || dloc1(1,1) > 0
                    activeModes(end+1) = ActiveMode("following", cars);
                    mlv_s = cars(Nc).state.x(2);
                else
                    mlv_s = -1;
                end
            elseif Nc > 1
                % sort the cars from farthest behind to farthest ahead.
                [s0,idx] = sort(dloc0(:,1));
                s1 = dloc1(idx,1);
                cars = cars(idx);

                ds0 = diff(s0);
                ds1 = diff(s1);

                ds = min(ds0, ds1);
            
                % if the farthest car behind us is going faster than us,
                % then add a following mode for it.
                if cars(1).state.x(2) > ego_speed
                    activeModes(end+1) = ActiveMode("following", cars(1));
                end
                
                % add a following or merging mode between all other cars,
                % if there is space.
                
                for i = 1:Nc-1
                    free_space = ds(i) - cars(i).Length/2 - cars(i+1).Length/2 - obj.ego_Length;
                    if free_space > obj.Max_Tight_Merge_Offset
                        activeModes(end+1) = ActiveMode("following", cars(i+1)); %#ok<AGROW>
                    elseif free_space > obj.Min_Tight_Merge_Offset
                        send(Q,'Attempt Merging!')
                        activeModes(end+1) = ActiveMode("merging", [cars(i), cars(i+1)]); %#ok<AGROW>
                    end
                end
                
                mlv_s = min(car_speed(any([dloc0(:,1),dloc1(:,1)]>0)));
            end
            
            search_mode = SearchMode(activeModes, obj.Goal_Lane + left_right, mlv_s);
        end
        
        function [search_mode, try_switch] = Search_Center(obj, dloc0, dloc1, car_lane, car_speed, cars, set_speed, Q)
            % Always use velocity keeping
            activeModes = ActiveMode("velocity_keeping",set_speed);
                
            % Look for a car in our lane and in front of us
            in_lane = car_lane == obj.Goal_Lane;

            s0 = dloc0(in_lane,1);
            car_speed = car_speed(in_lane);
            cars = cars(in_lane);
            
            nf = 99999;
            nfi = 0;
            nb = -99999;
            nbi = 0;
            for i = 1:numel(s0)
                if s0(i) > 0 && s0(i) < nf
                    nf = s0(i);
                    nfi = i;
                elseif s0(i) <= 0 && s0(i) > nb
                    nb = s0(i);
                    nbi = i;
                end
            end
            
            mlv_s = min(car_speed(s0>0));
            
            try_switch = false;
            if nfi ~= 0
                % there is a car infront
                if nbi ~= 0
                    % there is also a car in back
                    free_space = (nf - nb)  - cars(nbi).Length/2 - cars(nfi).Length/2 - obj.ego_Length;
                    if free_space > obj.Max_Tight_Merge_Offset
                        activeModes(end+1) = ActiveMode("following", cars(nfi));
                    else
                        activeModes(end+1) = ActiveMode("merging", [cars(nbi), cars(nfi)]);
                    end
                else
                    activeModes(end+1) = ActiveMode("following", cars(nfi));
                end
                
                try_switch = cars(nfi).state.x(2) < set_speed;
            end
                
            
%             in_lane_in_front = any([dloc0(:,1), dloc1(:,1)] > 0, 2) & ...
%                 any(abs([dloc0(:,2), dloc1(:,2)]) <= obj.Lane_Width/2, 2);
%             
%             try_switch = false;
%             
%             mlv_s = -1;
%             if any(in_lane_in_front)
%                 % Get the closest car. (at time 0)
%                 cars = cars(in_lane_in_front);
%                 
%                 [~, closest_idx] = min(dloc0(in_lane_in_front,1));
% %                 send(Q, cars(closest_idx))
%                 activeModes(end+1) = ActiveMode("following", cars(closest_idx));
% 
%                 % If the car is slower than the Goal_Speed -at
%                 % Behavior_Horizon- then try and switch lanes.
%                 try_switch = cars(closest_idx).state.x(2) < set_speed;
% 
%                 mlv_s = min(car_speed(in_lane_in_front)); % minimum speed infront
%             end
            search_mode = SearchMode(activeModes, obj.Goal_Lane, mlv_s);
        end
    end
end

% function y = sigmoid(x)
% y = 1/(1+exp(-x));
% end