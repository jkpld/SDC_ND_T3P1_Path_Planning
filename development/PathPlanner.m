classdef PathPlanner
    properties
        
        Time_Step = 0.02; % [s]
        Replanning_Time = 0.6; % [s]
        Plan_Horizon = 1; % [s]
        Behavior_Horizon = 2; % [s]
        Collision_Avoidance = 0.1; % [s]

        Consider_Distance = 50; % [m]
        
        Goal_Speed = 20; % [m/2]
        Lane_Width = 4; % [m]
        Num_Lanes = 3;
        
        end_state_s
        end_state_d
        
        ego(1,1) Vehicle
        trajGen(1,1) TrajectoryGeneration
        behavior % BehaviorModule
        
        Q
        
        counter = 1;
    end
    methods
        function OO = PathPlanner(Q)
            % Set ego's ID - not that it is used anywhere
            OO.ego.ID = 0;
            OO.Q = Q;
            
            OO.behavior = BehaviorModule(OO.ego, ...
                OO.Behavior_Horizon, ...
                OO.Consider_Distance, ...
                OO.Lane_Width, ...
                OO.Num_Lanes, ...
                OO.Goal_Speed, ...
                OO.trajGen);
        end
        
        function [state_s, state_d] = Frenet_to_Cartesian(~, state_x, state_y)
            state_s = state_x; % road is straight
            state_d = state_y; % road is straight
        end
        
        function [state_x, state_y] = Cartesian_to_Frenet(~, state_s, state_d)
            state_x = state_s; % road is straight
            state_y = state_d; % road is straight
        end
        
        function [path_xy, other_paths, OO] = GeneratePath(OO, ego_pose, path_xy, cars, other_paths)
            
            pointsRemaining = size(path_xy,1);
            t_end = pointsRemaining * OO.Time_Step;
            
            % Check to see if we need to replan the trajectory.
            if t_end < OO.Replanning_Time
                
%                 send(OO.Q, sprintf('------ Iteration %d ---------\n', OO.counter));
                
                % Set the initial state for the behavior/path-planning process
                if t_end == 0
                    % If there are no points in the path, then get the state
                    % information from ego_pose.
                    
                    state_x = [ego_pose.x, ego_pose.speed*cos(ego_pose.yaw), 0]; %[x, x_dot, x_dot_dot]
                    state_y = [ego_pose.y, ego_pose.speed*sin(ego_pose.yaw), 0];

                    % ----------------------------------
                    % Convert from Cartesian to Frenet
                    % ----------------------------------
                    [state_s, state_d] = Frenet_to_Cartesian(OO, state_x, state_y);
                else
                    % Set the ego state to the last one provided. The sd
                    % location in end_state_s and end_state_d should be the
                    % same as in ego_path_xy(end,:).
                    
                    state_s = OO.end_state_s;
                    state_d = OO.end_state_d;
                end
                
                % Set ego's state
                OO.ego.state = State(state_s, state_d);
                OO.ego.t0 = 0;

                % ---------------------------------------------------
                %  Behavior planning
                % ---------------------------------------------------

                
%                 % Predict other car locations at t_end and (t_end +
%                 % Plan_Horizon)
%                 cars1 = cars;
                for j = 1:numel(cars)
                    [states_x, states_y] = cars(j).generate_states(t_end);
                    cars(j).state = State(states_x(1,:),states_y(1,:));
                    cars(j).t0 = 0;
                    
%                     cars1(j).state = State(states_x(2,:),states_y(2,:));
%                     cars1(j).t0 = 0;                    
                end
                
                
                [OO.behavior, prop_ego, collide, other_trajs] = OO.behavior.PlanPath(OO.ego, cars, OO.Q);
                send(OO.Q, sprintf('Goal lane : %d\,', OO.behavior.Goal_Lane))
                
%                 [ego1_s, ego1_d] = OO.ego.generate_states(OO.Behavior_Horizon);
%                 
%                 % Set ego to just try and stay at a constant speed of 20 m/s
%                 activeModes = struct('name',"velocity_keeping",'data',min(ego_pose.speed+10, OO.Goal_Speed));
% 
%                 % Determine if there is a car infront of us
%                 FOLLOW_DIST = 20;
%                 min_car_idx = nearest_car_in_lane(OO, ego1_s(1), ceil(ego1_d(1)/OO.Lane_Width), cars1, FOLLOW_DIST);
%                 if ~isempty(min_car_idx)
%                     activeModes(end+1).name = "following";
%                     activeModes(end).data = cars(min_car_idx);
%                     
%                     % If there is a slow car infront, then try to changes
%                     % lanes.
%                     if cars1(min_car_idx).state.x(2) < OO.Goal_Speed
%                         % try to change lanes!
%                         send(OO.Q,'Try to change lanes!')
%                     end
%                 end
%                 
%                 % Start the path planning
%                 [prop_ego, collide] = OO.trajGen.generate(OO.Q, OO.ego, cars, activeModes);

                other_paths = cell(1,numel(other_trajs));
                for i = numel(other_trajs):-1:1
                    [states_s, states_d] = other_trajs(i).generate_states(OO.Time_Step:OO.Time_Step:OO.Plan_Horizon);
                    other_paths{i} = [states_s(:,1), states_d(:,1)];
                end

                [states_s, states_d] = prop_ego.generate_states(OO.Time_Step:OO.Time_Step:OO.Plan_Horizon);
                [states_x, states_y] = Cartesian_to_Frenet(OO, states_s, states_d);
                
                OO.end_state_s = states_s(end,:);
                OO.end_state_d = states_d(end,:);
                
                % Add the new states to the old ones
                path_xy = [path_xy; [states_x(:,1), states_y(:,1)]];
                
                OO.counter = OO.counter + 1;
            end
            
            
        end

        function min_car_idx = nearest_car_in_lane(OO, s, lane, cars, min_dist)
            lane = OO.Lane_Width*(lane-0.5);
            min_car_idx = [];
            for j = 1:numel(cars)
                
                if abs(cars(j).state.y(1) - lane) < OO.Lane_Width/2
                    dist = cars(j).state.x(1) - s;
                    if dist > 0 && dist < min_dist
                        min_dist = dist;
                        min_car_idx = j;
                    end
                end
            end
        end
    end
end