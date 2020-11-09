% script_test_fcn_drawTire.m - a script to test the function which
% draws a tire
close all
clc;


%% Test 1 - plain tire
% Set up the tire information
tire = fcn_tire_initTire;
fcn_drawTire(tire,1);

%% Test 2 - Change the orientation angle

% Set up the tire information
tire = fcn_tire_initTire;

tire.orientation_angle = 45;
fcn_drawTire(tire,1);
drawnow;

for angle = 0:1:180
    tire.orientation_angle = angle*pi/180; % the angle of the tire [rad]
    fcn_drawTire(tire,1);
    drawnow;
end

%% Test 3 - Change the rolling angle (only works for tire type = 2 or higher)

% Set up the tire information
tire = fcn_tire_initTire;

for angle = 0:1:180
    tire.rolling_angle = angle*pi/180; % the angle of the tire [rad]
    fcn_drawTire(tire,1);
    xlabel('x');
    ylabel('y');
    drawnow;
end


%% Test 4 - Change the tire color via direct color call

% Set up the tire information
tire = fcn_tire_initTire;
color_map = jet(256);
for color_ind = 1:256
    plot_color = color_map(color_ind,:);
    fcn_drawTire(tire,plot_color,1);
    drawnow;
end


%% Test 5 - Change the tire color via percentage force

% Set up the tire information
for percentage = 0:0.01:1
    fcn_drawTire(tire,percentage,1);
    drawnow;
end

%% Test 6 - Change the tire color via usage

% Set up the tire information
for percentage = 0:0.01:1
    tire.usage = percentage;
    fcn_drawTire(tire,1);
    drawnow;
end
