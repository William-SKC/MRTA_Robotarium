function waypoints = get_waypoints(start_id, end_id, paths, agent_to_task)
   


    if agent_to_task 
        id_str = "x_" + int2str(start_id-1)+ "_" +  int2str(end_id-1) + "_";
        path_str = paths.agent_to_task_paths.(id_str);
    else
        if start_id < end_id
            id_str = "x_" + int2str(start_id-1)+ "_" +  int2str(end_id-1) + "_";
        else
            id_str = "x_" + int2str(end_id-1)+ "_" +  int2str(start_id-1) + "_";
        end 
        path_str = paths.task_to_task_paths.(id_str);
    end
    path_str = strrep(path_str, '(', '');
    path_str = strrep(path_str, ')', '');
    path_str = strrep(path_str, '->', '');
    
    
    % Split the string by commas and spaces
    splitStr = regexp(path_str, '[, ]+', 'split');
    
    % Convert the string parts to numbers
    numbers = str2double(splitStr);
    
    % Reshape the array to 2-by-n format
    coordinates = reshape(numbers, 2, []);
    coordinates = (coordinates/100);
    coordinates(1,:) = coordinates(1,:)-1.2;
    coordinates(2,:) = coordinates(2,:)-0.8;
    waypoints = coordinates;
end