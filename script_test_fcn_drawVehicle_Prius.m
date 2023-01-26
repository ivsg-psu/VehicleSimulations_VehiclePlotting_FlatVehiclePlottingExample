% script_test_fcn_drawVehicle_Prius.m - a script to test the function which
% draws a vehicle

close all;
clc;


%% Set up the vehicle dimensions
vehicle = fcn_vehicle_initVehicle;
fcn_drawVehicle_Prius(vehicle,1);


%% Test 2 - Change the yaw angle of the vehicle
vehicle = fcn_vehicle_initVehicle;
for angle = 0:1:180
    vehicle.yawAngle_radians = angle*pi/180; % the yaw angle of the vehicle [rad]
    fcn_drawVehicle_Prius(vehicle,1);
    drawnow;
end

%% Test 3 - Change the steering angle
vehicle = fcn_vehicle_initVehicle;
for angle = -45:1:45
    vehicle.steeringAngle_radians = angle*pi/180; % the angle of the tire [rad]
    fcn_drawVehicle_Prius(vehicle,1);
    drawnow;
end

%% Test 4 - Change the rolling angle
vehicle = fcn_vehicle_initVehicle;
if vehicle.tire_type ~= 1
    vehicle = fcn_vehicle_initVehicle;
    for angle = 0:2:180
        for i_tire = 1:4
            vehicle.tire(i_tire).rolling_angle = angle*pi/180;
        end
        fcn_drawVehicle_Prius(vehicle,1);
        drawnow;
    end
end

%% Test 5 - Change the tire usage
vehicle = fcn_vehicle_initVehicle;
for usage = 0:0.01:1
    for i_tire = 1:4
        vehicle.tire(i_tire).usage = usage;
    end
    fcn_drawVehicle_Prius(vehicle,1);
    drawnow;
end

%% Test 6 - Change the position
vehicle = fcn_vehicle_initVehicle;
for X = 0:0.1:100
    vehicle.position_x = X;
    axis([X-10 X+10 -5 5]);
    
    fcn_drawVehicle_Prius(vehicle,2);
    drawnow limitrate    
end


%% Test 7 - Animate a trajectory
close all;
vehicle = fcn_vehicle_initVehicle;
load('ExampleTrajectory');
current_fig = figure(2);
clf;
% Makes the figure full screen
% set(current_fig,'units','normalized','outerposition',[0 0 1 1]);
plot(X,Y,'r-','Linewidth',3);

grid on;
% axis equal
ylim([-10 +10]);

% Draw the vehicle to start
fcn_drawVehicle_Prius(vehicle,2);

% Loop through time, animating as we move along
tic;
start = toc;
for i_time = 1:10:length(t)
    vehicle.position_x = X(i_time);
    vehicle.position_y = Y(i_time);
    vehicle.yawAngle_radians      = phi(i_time);
    vehicle.steeringAngle_radians = -df(i_time); % Steering is opposite direction in SAE vs ISO coordinates
    
    for i_tire = 1:4
        vehicle.tire(i_tire).rolling_angle = vehicle.position_x/(vehicle.tire(i_tire).length/2);
    end

    %     xlim([X(i_time)-10 X(i_time)+10]);
    %     ylim([-10 +10]);
    axis([X(i_time)-10 X(i_time)+10 -10 +10]);
        
    now = toc;
    if (now-start)>0.1
        fcn_drawVehicle_Prius(vehicle,2);
        drawnow limitrate
        start = now;
        % pause(0.01);
    end
    
end
fcn_drawVehicle_Prius(vehicle,2);

%%


function vehicle = fcn_vehicle_initVehicle
vehicle.width = 69.3/(12*3.281); % the width of the vehicle, [m] from 63.9 inches
vehicle.length = 106.3/(12*3.281); % the length of the vehicle, [m]
vehicle.wheel_width =  12.5/(12*3.281);  % the width of the wheel [m], assuming 12.5 inch width and 3.281 feet in a meter
vehicle.wheel_length = 33/(12*3.281);  % the diameter of the wheel [m], assuming 12.5 inch width and 3.281 feet in a meter
vehicle.front_axle = 1.4; % Location from the CG to the front axle [m]

% Use a plain tire - no spokes (see fcn_drawTire for details)
vehicle.tire_type = 3;

% Fill in tire information
starter_tire = fcn_tire_initTire;
starter_tire.usage = [];
for i_tire = 1:4    
    vehicle.tire(i_tire)= starter_tire;
end


vehicle.yawAngle_radians = 0; % the yaw angle of the body of the vehicle [rad]
vehicle.position_x = 0; % the x-position of the vehicle [m]
vehicle.position_y =0; % the y-position of the vehicle [m]
vehicle.steeringAngle_radians = 0; % the steering angle of the front tires [rad]
end


