function tire = fcn_tire_initTire(varargin)
% A Function that initializes a tire structure


% Tire dimensions (Prius uses a P215/45R17)
tire.width =  0.215;  % the width of the wheel [m], given by tire rating
tire.length = 17/(12*3.281) + 2*(0.45)*tire.width;  % the diameter of the wheel [m], 17 inches as wheel diameter and 45% aspect ratio

    
% % Set up the tire dimensions
% tire.width =  12.5/(12*3.281);  % the width of the wheel [m], assuming 12.5 inch width and 3.281 feet in a meter
% tire.length = 33/(12*3.281);  % the diameter of the wheel [m], assuming 12.5 inch width and 3.281 feet in a meter

% Default positions, etc.
tire.position_x = 0; % the x-position of the tire [m]
tire.position_y = 0; % the y-position of the tire [m]
tire.position_z = tire.length/2; % the z-position of the tire [m]
tire.rolling_angle = 0*pi/180; %
tire.orientation_angle = 0*pi/180; % the angle of the tire [rad]
tire.name = 'FrontRight'; % a string (no spaces) representing the tire's name
end