%% Setup simulation
% Other cars
cars(5) = Vehicle();
cars(1).state = State([-5,12,0],[10,0,0]);
cars(2).state = State([12,13,0],[6,0,0]);
cars(3).state = State([0,14,0],[10,0,0]);
cars(4).state = State([15,14,0],[10,0,0]);

cars(5).Length = 7;
cars(5).state = State([17,2,0.2],[2,0,0]);

cs = CarSim(cars);

% our car will start in center lane with 0 speed
ego = Vehicle();
ego.state = State([0, 0, 0], [6, 0, 0]);

activeModes = struct('name',"velocity_keeping",'data',20);
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

% Reset simulation
reset(cs)
cs.cars(2).state = State([12,13,0],[6,0,0]);
% Reset ego state;
ego.state = State([0, 0, 0], [6, 0, 0]);
ego.t0 = 0;

% Set counters
counter = 1;
counter2 = 1;

% Define planning time constants
PLAN_HORIZON = 1;
COLLISION_AVOIDANCE = 0.1;

% Set flags
computing = false;
computing_type = false;
Force_Compute = true;

% Start timer
start = tic;
add_collision = true;
while counter < 500

    
    
    if ~computing 
        
        if ((cs.t - ego.t0) > PLAN_HORIZON) || Force_Compute
            
            % Behavior planner should go here (maybe)
            
%         activeModes(2).data = cs.cars(2);
            ego.state = ego.state_at(cs.t);
            ego.t0 = cs.t;
            ego.last_collision_check_t = cs.t;

            f = parfeval(pool, gen_fun, 1, ego, activeModes, cs.cars);
            computing = true;
            computing_type = true;
            Force_Compute = false;
            
%         elseif (cs.t - ego.last_collision_check_t) > COLLISION_AVOIDANCE
%             
%             ego.last_collision_check_t = cs.t;
%             f = parfeval(pool, reactive, 3, ego, cs.cars);
%             computing = true;
%             computing_type = false;
        end
        
    elseif computing && strcmp(f.State,'finished')
        if computing_type
            [completedIdx, ego] = fetchNext(f, 0.01);
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
    
    if cs.advance(ego)
        break;
    end
    pause(0.02);
    
    counter = counter + 1;
end






















