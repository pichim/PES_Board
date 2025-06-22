function data = read_sdcard_data_with_time(file_name)
    %% open, read the file and close it
    
    fprintf('   --- read_sdcard_data_with_time ---\n');

    % open the file
    file_id = fopen(file_name); % test measurement
    
    % extract number of floats
    num_of_floats = fread(file_id, 1, 'uint8');
    fprintf('   Number of floats: %d\n', num_of_floats);
    
    % extract raw data
    data_raw = fread(file_id, 'single');
    fprintf('   Raw data length: %d\n', length(data_raw));
    
    % close the file
    fclose(file_id);


    %% preprocess the data
    
    % we assume delta time in microseconds is always the first meassurement
    
    % cut data to lenght
    data_raw = data_raw(1:floor( length(data_raw)/num_of_floats ) * num_of_floats);
    
    % reshape into a matrix
    data.values = reshape(data_raw, [num_of_floats, length(data_raw)/num_of_floats]).';
    
    % extract time
    data.time = cumsum(data.values(:,1)) * 1e-6;
    data.time = data.time - data.time(1);
    
    % extract values
    data.values = data.values(:,2:end);
    fprintf('   Data matrix: %dx%d\n', size(data.values));

    
end

