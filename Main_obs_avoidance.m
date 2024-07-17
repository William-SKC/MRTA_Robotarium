%% Demo code for robotarium
% Written by : Shengkang Chen      
% Date : 05-17-2024

%% Get input data 
close all
clear all
clc

folder = "problem_set_r3_t8_o2_robotarium/";

% Read problem file
problem_name = "problem_00001";
fname = folder + problem_name + ".json"; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
pro = jsondecode(str);
% save(problem_name+'.mat', 'pro');
% load(problem_name+'.mat', 'pro');

% Robots info
N = pro.num_agents; % number of robots
agents = pro.agents;
init_start_pos =  zeros(3, N);
init_start_pos(1,:) = extractfield(agents, 'x');
init_start_pos(2,:) = extractfield(agents, 'y');

% Convert from [0, 100] to [-1, 1]
init_start_pos(1:2,:) = (init_start_pos(1:2,:)/100);
init_start_pos(1,:) = init_start_pos(1,:)-1.2;
init_start_pos(2,:) = init_start_pos(2,:)-0.8;

% Tasks info
M = pro.num_tasks;
tasks = pro.tasks;
task_locations = zeros(2, M);
task_locations(1,:) = extractfield(tasks, 'x');
task_locations(2,:) = extractfield(tasks, 'y');
% Convert from [0, 100] to [-1, 1]
task_locations = (task_locations/100);
task_locations(1,:) = task_locations(1,:)-1.2;
task_locations(2,:) = task_locations(2,:)-0.8;
durations = pro.durations;
durations_vec = extractfield(durations, 'duration');

% Obstacle info
N_o = pro.num_obstacles;
obstacles = pro.obstacles;
obs_locations = zeros(2, N_o);
obs_locations(1,:) = extractfield(obstacles, 'x');
obs_locations(2,:) = extractfield(obstacles, 'y');
obs_locations = (obs_locations/100);
obs_locations(1,:) = obs_locations(1,:)-1.2;
obs_locations(2,:) = obs_locations(2,:)-0.8;

% Read schedule file
fname = folder + problem_name + "_schedule.json"; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 

% Remove unwanted characters
str = strrep(str, '(', '');  % Remove opening parentheses
str = strrep(str, ')', '');  % Remove closing parentheses
str = strrep(str, ' ', '');  % Remove spaces

% Split the string into a cell array of numbers
numbers = strsplit(str, ',');

% Convert the cell array to a numeric array
numbers = str2double(numbers);
robot_task_pairs = reshape(numbers, 2, []).';

save(problem_name+'_schedule.mat', 'robot_task_pairs');
load(problem_name+'_schedule.mat')

% Initialize a struct
assigned_goals = cell(1, N);


for j = 1:M
   robot_id = robot_task_pairs(j,2)+1;
   task_id = robot_task_pairs(j,1)+1;
   current_assignemnt = assigned_goals{robot_id};
   current_assignemnt = [current_assignemnt,task_id];
   assigned_goals{robot_id} = current_assignemnt;
end 

% Read path file
fname = folder + problem_name + "_path.json"; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
paths = jsondecode(str);


%% Set up Robotarium object
initial_positions = generate_initial_conditions(N, 'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);
disp("experiment started")
disp(fix(clock))

%Run the simulation for a specific number of iterations
iterations = 5000;
iterationsInitial = 900;

%% Set up constants for experiments
% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

lambda = 0.05;                  % Projection Distance
safety = 0.1 *r.robot_diameter;  % Safety Radius


%% Tools to map single-integrator -> unicycle

% Get the tools we need to map from single-integrator
[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping('ProjectionDistance', lambda);

% Create a barrier certificate for use with the above parameters 
unicycle_barrier_certificate = create_uni_barrier_certificate_with_boundary();

args = {'PositionError', 0.025, 'RotationError', 2};
init_checker = create_is_initialized(args{:});
controller = create_waypoint_controller(args{:});


%% Experiment start

% Get initial location data for while loop condition.
x=r.get_poses();
r.step();

radius = 0.05*ones(1, M);
viscircles(task_locations', radius);
text_offset = 0;
for i = 1:M
    text(task_locations(1, i)+text_offset, task_locations(2, i)+text_offset, int2str(i));
end 

for j = 1:N_o
    viscircles(obs_locations(:,j)', obstacles(j).radius*0.01);
end 

while(~init_checker(x, init_start_pos))
    x = r.get_poses();
    dxu = controller(x, init_start_pos);
    dxu = unicycle_barrier_certificate(dxu, x);      
    r.set_velocities(1:N, dxu);
    r.step();   
end


goal_points = zeros(3,N);
goal_points(1:2, :) = init_start_pos(1:2,:);
robots_traj = cell(N, 1);
for i = 1:N
    robots_traj{i} = goal_points(1:2, i);
end 

completed_tasks_vec = zeros(1,M);
current_task_ids = zeros(N,1);
waypoint_indices = ones(N,1)*2;
current_task_execution_time = zeros(N,1);
t = 0;
hText = text(-1.2, -0.8, "time", 'FontSize', 16);
colors = ['r','g', 'b', 'y'];
for i = 1:N
    if ~isempty(assigned_goals{i})
        task_id = assigned_goals{i}(1);
        robots_traj{i} = get_waypoints(i, task_id, paths, 1);
        plot(robots_traj{i}(1,:), robots_traj{i}(2,:), colors(i), 'LineWidth',5)
        current_task_ids(i) = task_id;
        goal_points(1:2,i) = robots_traj{i}(:, waypoint_indices(i));
    end
end

while sum(completed_tasks_vec) < M
    for i = 1:N
        task_id = current_task_ids(i);
        if norm(x(1:2, i)-goal_points(1:2,i)) < 0.05
            if waypoint_indices(i) < size(robots_traj{i}, 2)
                waypoint_indices(i) = waypoint_indices(i)+1;
                goal_points(1:2,i) = robots_traj{i}(:, waypoint_indices(i));
            else
                current_task_execution_time(i) = current_task_execution_time(i) +1;
                if current_task_execution_time(i) > durations_vec((i-1)*M+task_id)/r.time_step
                    current_task_execution_time(i) = 0;
                    completed_tasks_vec(task_id) = 1;
                    disp(task_id)
                    prev_task_id = task_id;
                    assigned_goals{i}(1) = [];
                    if ~isempty(assigned_goals{i})
                        task_id =  assigned_goals{i}(1);
                        current_task_ids(i) = task_id;
                        waypoint_indices(i) = 2;
                        robots_traj{i} = get_waypoints(prev_task_id, task_id, paths, 0);
                        goal_points(1:2,i) = robots_traj{i}(:, waypoint_indices(i));
                        plot(robots_traj{i}(1,:), robots_traj{i}(2,:), colors(i), 'LineWidth',5)   
                    end
                end
            end
        end 
    end
    
    x = r.get_poses();

    controller = create_waypoint_controller(args{:});

    dxu = controller(x, goal_points);
    dxu = unicycle_barrier_certificate(dxu, x);
    r.set_velocities(1:N, dxu);
    r.step();
    t = t + 1;
    newText = sprintf('Time: %f sec', t*r.time_step);  % Create new text content
    set(hText, 'String', newText);     % Set new text content

end
disp(t*r.time_step)
% for i = 1:N
%    plot(robots_traj{i}(1,:), robots_traj{i}(2,:), 'LineWidth',5)
% end 

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();