%% Setup simulation

% Initialize our car ("ego") to start in center lane with 0 speed
ego = Vehicle();
ego.state = State([-25, 0, 0], [2, 0, 0]);

% Other cars

% % Simulation 0 =================================================
cars_S0 = Vehicle.empty();

% Simulation 1 =================================================
for i = 5:-1:1
    % Need to initialize in a loop so that the ID's are actually random.
    cars(i) = Vehicle();
    cars(i).ID = i;
end
cars(1).state = State( [-5, 12, 0], [10, 0, 0]);
cars(2).state = State( [12, 13, 0], [6,  0, 0]);
cars(3).state = State( [0,  14, 0], [10, 0, 0]);
cars(4).state = State( [15, 14, 0], [10, 0, 0]);
cars(5).state = State( [17, 12, 0], [2,  0, 0]);
cars(5).Length = 19; % Car 5 will be a truck

cars_S1 = cars;

% Simulation 2 =================================================
for i = 5:-1:1
    % Need to initialize in a loop so that the ID's are actually random.
    cars(i) = Vehicle();
    cars(i).ID = i;
end
cars(1).state = State( [-15, 14, 0], [10, 0, 0]);
cars(2).state = State( [18, 13, 0], [6,  0, 0]);
cars(3).state = State( [0,  14, 0], [10, 0, 0]);
cars(4).state = State( [15, 14, 0], [10, 0, 0]);
cars(5).state = State( [17, 12, 0], [2,  0, 0]);
cars(5).Length = 19; % Car 5 will be a truck

cars_S2 = cars;

% Simulation 3 =================================================
cars_per_lane = [5,5,5];
lane_speed = [15, 12, 18];
car_dist = 20;
car_dist_sig = 5;
clear cars
cars(sum(cars_per_lane)) = Vehicle();
counter = 1;
for i = 1:3
    for j = 1:cars_per_lane(i)
        cars(counter).state = State([(j-1)*car_dist + min(max(randn(1)*car_dist_sig,-10),10), lane_speed(i), 0], [(i-0.5)*4,0,0]);
        cars(counter).Length = cars(counter).Length + min(max(randn(1)*2,-3),3);
        counter = counter + 1;
    end
end

cars_S3 = cars;

cs = CarSim(cars_S3, ego);

q = parallel.pool.DataQueue;
afterEach(q, @disp);

%% Setup parallel pool for simulation
pool = parpool(1);

%% Run simulation and ego computation asycronously
path_planner = PathPlanner(q);
clc

reset(cs) % Reset simulation
counter = 1; % Set counter
computing = false; % Set flags
path_xy = []; % Other parameters
other_paths = {};
while counter < 1800

    % Run the simulation
    [collide, ego_pose, path_xy, cars, other_paths] = cs.advance(path_xy, other_paths);
    if collide, break, end
%     tmp_ = rand(200);
    
    if ~computing
        % Start main
        f = parfeval(pool, @path_planner.GeneratePath, 3, ego_pose, path_xy, cars, other_paths);
        computing = true;
        
    elseif computing && strcmp(f.State,'finished')
        % If we finished computing the get the result from worker
        
        % Collect the result
        [~, path_xy, other_paths, path_planner] = fetchNext(f, 0.01);
        computing = false;
    end

%     path_xy
    
    counter = counter + 1;
end






















