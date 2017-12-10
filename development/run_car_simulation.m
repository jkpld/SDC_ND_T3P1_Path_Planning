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

while counter < 2000

    % Run the simulation
    [collide, ego_pose, path_xy, cars] = cs.advance(path_xy);
    if collide, break, end
%     tmp_ = rand(200);
    
    if ~computing
        % Start main
        f = parfeval(pool, @path_planner.GeneratePath, 2, ego_pose, path_xy, cars);
        computing = true;
        
    elseif computing && strcmp(f.State,'finished')
        % If we finished computing the get the result from worker
        
        % Collect the result
        [~, path_xy, path_planner] = fetchNext(f, 0.01);
        computing = false;
    end

%     path_xy
    
    counter = counter + 1;
end






















