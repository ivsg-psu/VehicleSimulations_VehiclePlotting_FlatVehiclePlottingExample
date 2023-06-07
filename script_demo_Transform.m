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



% Plot a mapping van, sensor platform bar, and LIDAR
% NOTE: dimensions here are WRONG, just approximations!!!!

% Mapping van
length = 190*(1/12)*(1/3.281); % 190 inches times 1ft = 12inches times 1 meter = 3.281 feet
width = 72*(1/12)*(1/3.281); % 72 inches times 1ft = 12inches times 1 meter = 3.281 feet
height = 72*(1/12)*(1/3.281); % 72 inches times 1ft = 12inches times 1 meter = 3.281 feet
cube_points = fcn_INTERNAL_fillCube(length,width,height);
% Plot the result and make the body the parent
mapping_van_plot_handles = fcn_INTERNAL_plotCube(cube_points);

ax = gca; % Grab current axes;
handles.transform_ground_to_body = hgtransform('Parent',ax);
handles.transform_body_to_sensorplatform = hgtransform('Parent',handles.transform_ground_to_body);
handles.transform_sensorplatform_to_LIDAR = hgtransform('Parent',handles.transform_body_to_sensorplatform);

set(mapping_van_plot_handles,'Parent',handles.transform_ground_to_body);

% Sensor platform bar
length = 2*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
width = 60*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
height = 2.5*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
sensor_box_points = fcn_INTERNAL_fillCube(length,width,height);
% Plot the result and make the sensorplatform the parent
sensor_platform_plot_handles = fcn_INTERNAL_plotCube(sensor_box_points);
set(sensor_platform_plot_handles,'Parent',handles.transform_body_to_sensorplatform);


% LIDAR box
length = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
width = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
height = 10*(1/12)*(1/3.281); % inches times 1ft = 12inches times 1 meter = 3.281 feet
lidar_box_points = fcn_INTERNAL_fillCube(length,width,height);
% Plot the result and make the LIDAR the parent
lidar_plot_handles = fcn_INTERNAL_plotCube(lidar_box_points);
set(lidar_plot_handles,'Parent',handles.transform_sensorplatform_to_LIDAR);

disp('Note the locations prior to moving each of the objects to their correct locations!')
disp('Hit any key to continue...')
pause;

M = makehgtform('translate',[body_rotated_translated(i_tire,1) body_rotated_translated(i_tire,2) 0],'zrotate',pi/2);
set(transform_body_to_tire,'Matrix',M);


% Make handles to each of the plotted objects
h_vehicle_body = plot_box(0,0,body_rotated_translated,[0.1 0.1 0.1]);  % Body
h_vehicle_exterior_decorators = plot(exterior_decorators_rotated_translated(:,1),exterior_decorators_rotated_translated(:,2),'k-');
h_vehicle_interior_decorators = plot(interior_decorators_rotated_translated(:,1),interior_decorators_rotated_translated(:,2),'-','Color',[0.5 0.5 0.5]);

% Make all the handles be parents of a transform object (makes
% rendering fast)
set(h_vehicle_body,'Parent',handles.transform_ground_to_body);
set(h_vehicle_exterior_decorators,'Parent',handles.transform_ground_to_body);
set(h_vehicle_interior_decorators,'Parent',handles.transform_ground_to_body);

% Save the transform so we can use it again later
set(fig_num,'UserData',handles)

% Calculate the tire details
if flag_show_tire
    for i_tire = 1:length(vehicle.tire)

        % Are there tires at every corner?
        if length(vehicle.tire)~=length(body_rotated_translated(:,1))
            error('Number of tires must match corners of vehicle');
        end

        % Grab the tire
        tire = vehicle.tire(i_tire);

        % Steer the front tires (tires 1 and 2)?
        if i_tire>2
            tire.orientation_angle = 0; % the angle of the rear tires [rad]
        else
            tire.orientation_angle = vehicle.steeringAngle_radians; % the angle of the tire [rad]
        end


        % Draw the tire now to create the handle
        if isempty(vehicle.tire(i_tire).usage)
            fcn_drawTire(tire,[0 0 0],fig_num);
        else
            fcn_drawTire(tire,vehicle.tire(i_tire).usage,fig_num);
        end

        % Create a transform object for the body to tire position
        % transform
        transform_body_to_tire = hgtransform('Parent',handles.transform_ground_to_body);

        % Grab the handle that was just created, and tell it that it is
        % stuck to the vehicle's body
        handles = get(fig_num,'UserData');
        set(handles.h_tires.(tire.name).body_transform_object,'Parent',transform_body_to_tire);

        % Put the tire in the correct location on the vehicle's body,
        % in correct orientation
        M = makehgtform('translate',[body_rotated_translated(i_tire,1) body_rotated_translated(i_tire,2) 0],'zrotate',pi/2);
        set(transform_body_to_tire,'Matrix',M);

        % Save the result to the handles
        handles.h_tires.(tire.name).transform_body_to_tire = transform_body_to_tire;

        % Plot the usage?
        if ~isempty(vehicle.tire(i_tire).usage)
            if isnumeric(vehicle.tire(i_tire).usage)
                plot_str = convertToColor(vehicle.tire(i_tire).usage);
                set(handles.h_tires.(tire.name).body,'Color',plot_str);
            end
        end

    end
end % Ends if flag show tire

% Use makehgtform to translate and rotate the body
M = makehgtform('translate',[vehicle.position_x vehicle.position_y 0],'zrotate',vehicle.yawAngle_radians);
set(handles.transform_ground_to_body,'Matrix',M);

% Save the transform so we can use it again later
set(fig_num,'UserData',handles)

function cube_points = fcn_INTERNAL_fillCube(length,width,height)
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
end

function plot_handles = fcn_INTERNAL_plotCube(cube_points)
% See help on hgtransform to understand why we are saving the handles
% Plot the edges
plot_handles(1) = plot3(cube_points(1:end-1,1),cube_points(1:end-1,2),cube_points(1:end-1,3),'.-','MarkerSize',20);
color_plot = get(plot_handles(1),'Color');
hold on;
axis equal;
grid on;

% Plot the origin
plot_handles(2) = plot3(cube_points(end,1),cube_points(end,2),cube_points(end,3),'Color',color_plot,'MarkerSize',50);

% Plot the coordinate system
most_negative_point = cube_points(1,:);
most_positive_point = cube_points(end-2,:);
differences = most_positive_point-most_negative_point;

% Plot x-axis
plot_handles(3) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    differences(1),0,0,'Color',color_plot,'LineWidth',3);
% Plot y-axis
plot_handles(4) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    0,differences(2),0,'Color',color_plot,'LineWidth',3);
% Plot z-axis
plot_handles(5) = quiver3(cube_points(end,1),cube_points(end,2),cube_points(end,3),...
    0,0,differences(3),'Color',color_plot,'LineWidth',3);


end
