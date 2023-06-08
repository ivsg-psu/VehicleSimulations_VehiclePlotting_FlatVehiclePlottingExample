% script_demo_Transform.m
% This is a script to demonstrate transformation matrix operations.
% This function was originally written on 2023_06_07 by S. Brennan,
% sbrennan@psu.edu

% Revision history:
%     2023_06_07
%     -- first write of the code


% Create a transform object connecting ground to body, body to
% sensorplatform, sensorplatform to LIDAR_sensor, sensorplatform to GPS
% sensor. Save all the handles in a structure called "handles".

figure(1);
clf;
clear handles
plot([],[]); % Create an empty plot
xlim([-10 10]);
ylim([-10 10]);



% Plot a mapping van, sensor platform bar, and LIDAR
% NOTE: dimensions here are WRONG, just approximations!!!!
% Taken from: https://www.vanguide.co.uk/guides/ford-transit-connect-dimensions-guide/

% Mapping van
length = 4.825; % Meters
width = 2.137; % Meters
height = 1.827; % Meters

% Define offsets of the cube edges relative to the cube's center. Offsets
% are added onto all points except the origin.
offset_length = length/2 - 0.878; % Meters
offset_width = 0;
tire_offset_height = 15*(1/12)*(1/3.281); % 15 inches times 1 ft is 12 inches, times 1 meter is 3.281 feet.
offset_height = height/2 - tire_offset_height;
cube_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width,offset_height);
% Plot the result and make the body the parent
mapping_van_plot_handles = fcn_INTERNAL_plotCube(cube_points);

% Once the first plot is made, we can define the axes
ax = gca; % Grab current axes;
handles.transform_ground_to_body = hgtransform('Parent',ax);
handles.transform_body_to_sensorplatform = hgtransform('Parent',handles.transform_ground_to_body);
handles.transform_sensorplatform_to_LIDAR = hgtransform('Parent',handles.transform_body_to_sensorplatform);
set(mapping_van_plot_handles,'Parent',handles.transform_ground_to_body);

% Sensor platform bar
length = 2*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
width = 60*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
height = 2.5*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
% Define offsets of the cube edges relative to the cube's center. Offsets
% are added onto all points except the origin.
offset_length = 0; % Meters
offset_width = 0;
offset_height = height/2; 
sensor_box_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width,offset_height);
% Plot the result and make the sensorplatform the parent
sensor_platform_plot_handles = fcn_INTERNAL_plotCube(sensor_box_points);
set(sensor_platform_plot_handles,'Parent',handles.transform_body_to_sensorplatform);


% SICK LIDAR box
length = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
width = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
height = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
% Define offsets of the cube edges relative to the cube's center. Offsets
% are added onto all points except the origin.
offset_length = 0; % Meters
offset_width = 0;
offset_height = 0; 
lidar_SICK_box_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width,offset_height);
% Plot the result and make the LIDAR the parent
lidar_sick_plot_handles = fcn_INTERNAL_plotCube(lidar_SICK_box_points);
set(lidar_sick_plot_handles,'Parent',handles.transform_sensorplatform_to_LIDAR);

disp('Note the locations prior to moving each of the objects to their correct locations!')
disp('Hit any key to continue...')
pause(1);

%% Now, start moving items to their correct locations
% Move the sensor platform origin
sensor_box_offset_x_relative_to_body = -1; % Meters - GUESS!!
sensor_box_offset_y_relative_to_body =  0; % Meters - GUESS!!
sensor_box_offset_z_relative_to_body = 1.6; % Meters - GUESS!!

M1 = makehgtform('translate',[sensor_box_offset_x_relative_to_body sensor_box_offset_y_relative_to_body sensor_box_offset_z_relative_to_body]);
set(handles.transform_body_to_sensorplatform,'Matrix',M1);
fprintf(1,'\nThe matrix transform from body origin to sensor platform is: \n');
disp(M1);

% Move the SICK LIDAR to its correct location
lidar_sick_offset_x_relative_to_sensorplatform = -0.4; % Meters - GUESS!!
lidar_sick_offset_y_relative_to_sensorplatform =  0; % Meters - GUESS!!
lidar_sick_offset_z_relative_to_sensorplatform = -0.1; % Meters - GUESS!!

M2 = makehgtform('translate',[lidar_sick_offset_x_relative_to_sensorplatform, lidar_sick_offset_y_relative_to_sensorplatform, lidar_sick_offset_z_relative_to_sensorplatform]);
M3 = makehgtform('yrotate',90*pi/180);

set(handles.transform_sensorplatform_to_LIDAR,'Matrix',M2*M3);
fprintf(1,'\nThe matrix transform from sensor platform to SICK lidar is: \n');
disp(M2);

%% Show that we can rotate everything together by simply rotating the ground frame
for angles = (0:2:360)
    M0 = makehgtform('zrotate',angles*pi/180);
    set(handles.transform_ground_to_body,'Matrix',M0);
    pause(0.01);
end


%% Test transforms at the origin
% Take a measurement in global, convert to LIDAR
fprintf(1,'\n\nTEST POINT: ORIGIN\n');
body_fixed_measurement = [0 0 0];
plot3(body_fixed_measurement(1),body_fixed_measurement(2),body_fixed_measurement(3),'r.','MarkerSize',50);

fprintf(1,'\nTest point is located, in global frame, at:\n');
disp(body_fixed_measurement);

% Convert to homogenous coordinates
homogenous_measurement = [body_fixed_measurement 1]';

% Multiply through by the transform stacks:
% Use pseudo-inverse of the transform, M1, to calculate sensor platform
% pose. The negative is to indicate that the direction is from the sensor
% platform origin, to the point (not from point to the platform origin).
% For the assertions: we start to get numerical precision issues, so check
% agreement to 6 decimal places
sensorplatform_measurement = M1\homogenous_measurement; 
assert(isequal(round(sensorplatform_measurement(1),6), round(body_fixed_measurement(1) -sensor_box_offset_x_relative_to_body,6)));
assert(isequal(round(sensorplatform_measurement(2),6), round(body_fixed_measurement(2) -sensor_box_offset_y_relative_to_body,6)));
assert(isequal(round(sensorplatform_measurement(3),6), round(body_fixed_measurement(3) -sensor_box_offset_z_relative_to_body,6)));

fprintf(1,'Test point is located, in sensorplatform frame, at:\n');
disp(sensorplatform_measurement(1:3,1)');

% Multiply through by the transform stacks:
LIDAR_measurement = inv(M2*M3)*sensorplatform_measurement;
assert(isequal(round(LIDAR_measurement(1),6), round( -body_fixed_measurement(3) +     sensor_box_offset_z_relative_to_body + lidar_sick_offset_z_relative_to_sensorplatform,6)));
assert(isequal(round(LIDAR_measurement(2),6), round(  body_fixed_measurement(2) +     sensor_box_offset_y_relative_to_body + lidar_sick_offset_y_relative_to_sensorplatform,6)));
assert(isequal(round(LIDAR_measurement(3),6), round(  body_fixed_measurement(1) + -1*(sensor_box_offset_x_relative_to_body + lidar_sick_offset_x_relative_to_sensorplatform),6)));

% Now show that we can chain the multiplication of transforms, and get same
% result
LIDAR_measurement2 = (M1*M2*M3)\homogenous_measurement;
assert(isequal(round(LIDAR_measurement2,6),round(LIDAR_measurement,6)));

fprintf(1,'Test point is located, in SICK lidar frame, at:\n');
disp(LIDAR_measurement(1:3,1)');

% Now show how to go from sensor measurement to global
% Convert from sensor frames into global
body_fixed_measurement_recovered = M1*M2*M3*LIDAR_measurement;
assert(isequal(round(body_fixed_measurement,6),round(body_fixed_measurement_recovered(1:3,1)',6)));

fprintf(1,'Test point is located in global frame, using LIDAR measurement, at:\n');
disp(body_fixed_measurement_recovered(1:3,1)');

%% Test transforms as if it is a ground hit of the LIDAR directly under the sensor
% Convert from global into local frames
% Take a measurement in global, convert to LIDAR
fprintf(1,'\n\nTEST POINT: DIRECTLY UNDER SICK LIDAR\n');
body_fixed_measurement = [-1.4 0 -tire_offset_height];
plot3(body_fixed_measurement(1),body_fixed_measurement(2),body_fixed_measurement(3),'c.','MarkerSize',50);

fprintf(1,'\nTest point is located, in global frame, at:\n');
disp(body_fixed_measurement);

% Convert to homogenous coordinates
homogenous_measurement = [body_fixed_measurement 1]';

% Multiply through by the transform stacks:
% Use pseudo-inverse of the transform, M1, to calculate sensor platform
% pose. The negative is to indicate that the direction is from the sensor
% platform origin, to the point (not from point to the platform origin).
% For the assertions: we start to get numerical precision issues, so check
% agreement to 6 decimal places
sensorplatform_measurement = M1\homogenous_measurement; 
assert(isequal(round(sensorplatform_measurement(1),6), round(body_fixed_measurement(1) -sensor_box_offset_x_relative_to_body,6)));
assert(isequal(round(sensorplatform_measurement(2),6), round(body_fixed_measurement(2) -sensor_box_offset_y_relative_to_body,6)));
assert(isequal(round(sensorplatform_measurement(3),6), round(body_fixed_measurement(3) -sensor_box_offset_z_relative_to_body,6)));

fprintf(1,'Test point is located, in sensorplatform frame, at:\n');
disp(sensorplatform_measurement(1:3,1)');

% Multiply through by the transform stacks:
LIDAR_measurement = inv(M2*M3)*sensorplatform_measurement;
assert(isequal(round(LIDAR_measurement(1),6), round( -body_fixed_measurement(3) +     sensor_box_offset_z_relative_to_body + lidar_sick_offset_z_relative_to_sensorplatform,6)));
assert(isequal(round(LIDAR_measurement(2),6), round(  body_fixed_measurement(2) +     sensor_box_offset_y_relative_to_body + lidar_sick_offset_y_relative_to_sensorplatform,6)));
assert(isequal(round(LIDAR_measurement(3),6), round(  body_fixed_measurement(1) + -1*(sensor_box_offset_x_relative_to_body + lidar_sick_offset_x_relative_to_sensorplatform),6)));

% Now show that we can chain the multiplication of transforms, and get same
% result
LIDAR_measurement2 = (M1*M2*M3)\homogenous_measurement;
assert(isequal(round(LIDAR_measurement2,6),round(LIDAR_measurement,6)));

fprintf(1,'Test point is located, in SICK lidar frame, at:\n');
disp(LIDAR_measurement(1:3,1)');

% Now show how to go from sensor measurement to global
% Convert from sensor frames into global
body_fixed_measurement_recovered = M1*M2*M3*LIDAR_measurement;
assert(isequal(round(body_fixed_measurement,6),round(body_fixed_measurement_recovered(1:3,1)',6)));

fprintf(1,'Test point is located in global frame, using LIDAR measurement, at:\n');
disp(body_fixed_measurement_recovered(1:3,1)');




%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§


function cube_points = fcn_INTERNAL_fillCube(length,width,height,offset_length,offset_width, offset_height)
fprintf(1,'Preparing cube of:\n');
fprintf(1,'\tLength: %.2f\n',length);
fprintf(1,'\tWidth: %.2f\n',width);
fprintf(1,'\tHeigth: %.2f\n',height);
cube_points = [
    -length -width -height
    length -width -height
    nan     nan    nan
    -length  width -height
    length  width -height
    nan     nan    nan
    -length -width  height
    length -width  height
    nan     nan    nan
    -length  width  height
    length  width  height
    nan     nan    nan

    -length -width -height
    -length +width -height
    nan     nan    nan
    +length -width -height
    +length +width -height
    nan     nan    nan
    -length -width  height
    -length +width  height
    nan     nan    nan
    +length -width  height
    +length +width  height
    nan     nan    nan

    -length -width -height
    -length -width +height
    nan     nan    nan
    +length -width -height
    +length -width +height
    nan     nan    nan
    -length +width -height
    -length +width +height
    nan     nan    nan
    +length +width -height
    +length +width +height
    nan     nan    nan
    0 0 0
    ]*0.5;
cube_points(1:end-1,:) = cube_points(1:end-1,:)+[offset_length offset_width offset_height];
end

function plot_handles = fcn_INTERNAL_plotCube(cube_points)
% See help on hgtransform to understand why we are saving the handles
% Plot the edges
plot_handles(1) = plot3(cube_points(1:end-1,1),cube_points(1:end-1,2),cube_points(1:end-1,3),'.-','MarkerSize',20);
color_plot = get(plot_handles(1),'Color');
hold on;
axis equal;
grid on;
xlim([-10 10]);
ylim([-10 10]);

% Plot the origin
plot_handles(2) = plot3(cube_points(end,1),cube_points(end,2),cube_points(end,3),'Color',color_plot,'MarkerSize',50);

% Plot the coordinate system
most_negative_point = cube_points(1,:);
most_positive_point = cube_points(end-2,:);
differences = most_positive_point-most_negative_point;
mean_differences = mean(differences);
differences = [mean_differences mean_differences mean_differences];

% Plot x-axis
plot_handles(3) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    differences(1),0,0,'Color',color_plot,'LineWidth',3);
text_point = [cube_points(end,1),cube_points(end,2),cube_points(end,3)] + ...
    [differences(1),0,0];
plot_handles(4) = text(text_point(1),text_point(2),text_point(3),'X','Color',color_plot);

% Plot y-axis
plot_handles(5) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    0,differences(2),0,'Color',color_plot,'LineWidth',3);
text_point = [cube_points(end,1),cube_points(end,2),cube_points(end,3)] + ...
    [0,differences(2),0];
plot_handles(6) = text(text_point(1),text_point(2),text_point(3),'Y','Color',color_plot);

% Plot z-axis
plot_handles(7) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    0,0,differences(3),'Color',color_plot,'LineWidth',3);
text_point = [cube_points(end,1),cube_points(end,2),cube_points(end,3)] + ...
    [0,0,differences(3)];
plot_handles(8) = text(text_point(1),text_point(2),text_point(3),'Z','Color',color_plot);


end