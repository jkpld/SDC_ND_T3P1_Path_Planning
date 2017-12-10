%% Setup simulation

% Initialize our car ("ego") to start in center lane with 0 speed
ego = Vehicle();
ego.state = State([0, 0, 0], [6, 0, 0]);

% Other cars
cars(5) = Vehicle();
cars(1).state = State( [-5, 12, 0], [10, 0, 0]);
cars(2).state = State( [12, 13, 0], [6,  0, 0]);
cars(3).state = State( [0,  14, 0], [10, 0, 0]);
cars(4).state = State( [15, 14, 0], [10, 0, 0]);
cars(5).state = State( [17, 12, 0], [2,  0, 0]);
cars(5).Length = 19; % Car 5 will be a truck

cs = CarSim(cars, ego);

% activeModes(2).name = "following";
% activeModes(2).data = cars(2);

q = parallel.pool.DataQueue;
afterEach(q, @disp);

trajGen = TrajectoryGeneration();
gen_fun = @(ego, activeModes, cars) generate(trajGen, q, ego, activeModes, cars);
reactive = @(ego, cars) reactive_layer(trajGen, ego, cars, q);

% %%
% ego.state = State(0, [0, 0, 0], [6, 0, 0]);
% ego = trajGen.generate(ego, activeModes);
% ego2 = gen_fun(ego, activeModes);
%% Setup parallel pool for simulation
pool = parpool(1);

%% Run simulation and ego computation asycronously
clc

% Reset ego state;
ego.state = State([0, 0, 0], [6, 0, 0]);
ego.t0 = 0;

% Reset simulation
reset(cs, ego)
cs.cars(2).state = State([12,13,0],[6,0,0]);

% Set counters
counter = 1;
counter2 = 1;

% Define planning time constants
REPLANNING_TIME = 0.2;
PLAN_HORIZON = 1;
COLLISION_AVOIDANCE = 0.1;

TIME_STEP = 0.02;

% Set flags
computing = false;
computing_type = false;
Force_Compute = true;

% Start timer
start = tic;
add_collision = true;

% Other parameters
path_time_given = 0;
ego_path_xy = [];
states_s = [];
states_d = [];

Goal_Speed = 20;

while counter < 1800

    % Run the simulation
    [collide, ego_pose, ego_path_xy, cars] = cs.advance(ego_path_xy);
    if collide, break, end
    pause(0.017);
    
    % Compute trajectories asyncronously with the simulation. If not
    % currently computing, then see if we need to do anything...
    if ~computing
        
        % Time till end of previously planned trajectory
        N_path_points = size(ego_path_xy,1);
        t_end = N_path_points * TIME_STEP;
        
        
        % Check to see if we need to replan the trajectory.
        if (t_end < REPLANNING_TIME) || Force_Compute
            
            % Set the initial state for the behavior/path-planning process
            if t_end == 0
                % If there are no points in the path, then get the state
                % information from ego_pose.
            
                state_x = [ego_pose.x, ego_pose.speed*cos(ego_pose.yaw), 0]; %[x, x_dot, x_dot_dot]
                state_y = [ego_pose.y, ego_pose.speed*sin(ego_pose.yaw), 0];
                
                % ----------------------------------
                % Convert from Cartesian to Frenet
                % ----------------------------------
                
                state_s = state_x; % road is straight
                state_d = state_y; % road is straight
            else
                % Set the ego state to the last one provided. The sd
                % location in end_state_s and end_state_d should be the
                % same as in ego_path_xy(end,:).
                state_s = end_state_s;
                state_d = end_state_d;
            end
            
            ego.state = State(state_s,state_d);%ego.state_at(cs.t);
            ego.t0 = 0;
            
            % Predict other car locations at t_end
            cars_d = zeros(1,numel(cars));
            cars_s = zeros(1,numel(cars));
            for j = 1:numel(cars)
                cars(j).state = cars(j).state_at(t_end);
                cars(j).t0 = 0;
                cars_d(j) = cars(j).state.y(1);
                cars_s(j) = cars(j).state.x(1);
            end
            
            % The state/trajectory of our car (ego) and all other cars,
            % should now be relative to t=0 (current time). This is good
            % for the planning function
            
            % ---------------------------------------------------
            %  Behavior planning here (maybe)
            % ---------------------------------------------------
            
            % Set ego to just try and stay at a constant speed of 20 m/s
            activeModes = struct('name',"velocity_keeping",'data',min(ego_pose.speed+10, Goal_Speed));
            
            % find cars within say 80 m in s
            FOLLOW_DIST = 80;
            
            for j = 1:numel(cars)
                dist_s = cars(j).state.x(1) - state_s(1);
                dist_d = abs(cars(j).state.y(1) - state_d(1));
                % Cars in the same lane and within FOLLOW_DIST infront of
                % ego
                if (dist_d < 2.5) && (dist_s > 0) && (dist_d < FOLLOW_DIST)
                    activeModes(end+1).name = "following"; %#ok<SAGROW>
                    activeModes(end).data = cars(j);
                end
            end            
            
            % Start the path planning
            f = parfeval(pool, gen_fun, 1, ego, activeModes, cars);
            computing = true;
            computing_type = true;
            Force_Compute = false;
            
%         elseif (path_time_give - t_end) > COLLISION_AVOIDANCE
%             % Trim states_s/d to be the same size as ego_path_xy.
%             
%             path_time_given = t_end; % Reset the countdown 
%             
%             
%             f = parfeval(pool, reactive, 3, ego, cars);
%             computing = true;
%             computing_type = false;
        end
        
    elseif computing && strcmp(f.State,'finished')
        % If we finished computing the get the result from worker
        
        % If performing path planning
        if computing_type
            
            % Collect the result
            [~, ego] = fetchNext(f, 0.01);

            % Compute the new set of states. Note we need to start from
            % TIME_STEP because starting at 0 would end up duplicating the
            % end_state
            [states_s, states_d] = ego.generate_states(TIME_STEP:TIME_STEP:PLAN_HORIZON);
            
            end_state_s = states_s(floor(REPLANNING_TIME / TIME_STEP),:);
            end_state_d = states_d(floor(REPLANNING_TIME / TIME_STEP),:);
            f
%             end_state_s = states_s(end,:);
%             end_state_d = states_d(end,:);
            
            % ---------------------------------------------------
            %  CONVERT FROM FRENET TO CARTESIAN HERE
            % ---------------------------------------------------
            states_x = states_s; % road is straight
            states_y = states_d; % road is straight
            
            % Add the new states to the old ones
            ego_path_xy = [ego_path_xy; [states_x(:,1), states_y(:,1)]]; %#ok<AGROW>
            
            % Let the loop know we are done computing
            computing = false;
            fprintf('Finished path planning : %0.3f\n', toc(start));
            counter2 = counter2 + 1;
            
            % Add a collision to check responce
%             if counter2 > 5 && add_collision
%                 state = cs.cars(2).state;
%                 loc = ego.location(cs.t);
%                 state.x(1) = loc(1) + 10;
%                 cs.cars(2).state = state;
%                 cs.cars(2).t0 = cs.t;
%                 add_collision = false;
%             end
        else
            [completedIdx, collide, ego, dt] = fetchNext(f, 0.01);
            computing = false;
            fprintf('Finished collision detection (%d) : %0.3f\n', collide, dt);
                        
            if collide
                ego.t0 = cs.t; 
                Force_Compute = true;% force path planning on next iteration.
            end
        end
    end
    
    
    
    counter = counter + 1;
end






















