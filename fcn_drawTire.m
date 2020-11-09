function fcn_drawTire(tire,varargin)
% fcn_drawTire draws a tire in a given location.
% The user can specify a figure to use, or if none is specified, the
% current figure is used.
%
% FORMAT:
%
%          fcn_drawTire(tire,varargin)
%
% INPUTS:
%          tire:    a structure that contains the following fields:
%                   tire.theta - the yaw angle of the body of the tire
%                   [rad]
%                   tire.position_x - the x-position of the tire [m]
%                   tire.position_y - the y-position of the tire [m]
%                   tire.rolling_angle - angle of the tire's motion (e.g. what angle it has rolled forward) [rad]
%                   tire.orientation_angle - angle of the plane of the tire [rad]
%                   tire.name  - a string (no spaces) representing the tire's name
%
%                   tire.width - the width of the tire
%                   tire.length - the length of the tire
%
%          varargin: (option) the number of the figure where plotting is
%          occuring
%
% OUTPUTS:
%
% EXAMPLES:
%

%    % BASIC example - find all the points
%
%
% This function was written on 2020_11_02 by S. Brennan
% Questions or comments? sbrennan@psu.edu
%

% Revision history:
% 2020_11_02 - wrote the code


%% Set up for debugging
flag_do_debug = 0; % Flag to plot the results for debugging

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'Starting function: %s, in file: %s\n',st(1).name,st(1).file);
end

if flag_do_debug
    fig_num = 2;
    figure(fig_num);
    flag_make_new_plot = 1;
end



%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin < 1 || nargin > 4
    error('Incorrect number of input arguments.')
end

flag_make_new_plot = 0; % Default is not to make a new plot
if 2 <= nargin
    fig_num = varargin{end};
    figure(fig_num);
    
    % Check to see if the figure has user data within it already
    handles = get(fig_num,'UserData');
    if isempty(handles)
        flag_make_new_plot = 1;
    end
    try
        temp = handles.h_tires.(tire.name); %#ok<NASGU>
    catch
        flag_make_new_plot = 1;
    end
    
else
    fig = gcf; % create new figure with next default index
    fig_num = get(fig,'Number');
    flag_make_new_plot = 1;
end

% Check to see if a plot type is given?
plot_str = 'k-';
if 3 == nargin
    plot_str = varargin{1};
end

vehicle_handle = [];
if 4 == nargin
    vehicle_handle = varargin{2};
end

%% Start the main code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tire_type = 1;

switch tire_type
    case 1 % Simple square tire
        plotSimpleTire(tire,flag_make_new_plot,fig_num,plot_str,vehicle_handle);
    case 2  % Simple 3D spokes - may cause the image to jump when animated
        plotSpokeTire(tire, flag_make_new_plot,fig_num,plot_str,vehicle_handle);
    case 3  % Mesh tire
        plotMeshTire(tire, flag_make_new_plot,fig_num,plot_str,vehicle_handle);
    otherwise
        warning('Unknown tire type selected');
end


end % Ends the function



%% Functions for plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    _____ _                 _   _______ _          
%   / ____(_)               | | |__   __(_)         
%  | (___  _ _ __ ___  _ __ | | ___| |   _ _ __ ___ 
%   \___ \| | '_ ` _ \| '_ \| |/ _ \ |  | | '__/ _ \
%   ____) | | | | | | | |_) | |  __/ |  | | | |  __/
%  |_____/|_|_| |_| |_| .__/|_|\___|_|  |_|_|  \___|
%                     | |                           
%                     |_|                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plotSimpleTire(tire,flag_make_new_plot,fig_num,plot_str,vehicle_handle)

if flag_make_new_plot
    % Define the coordinates of the wheels (just a square for now). Start with
    % the front left side of the right wheel, and go clockwise. Then repeat for
    % the left wheel
    tire_box = [...
        -tire.width/2  tire.length/2;
        +tire.width/2  tire.length/2;
        +tire.width/2 -tire.length/2;
        -tire.width/2 -tire.length/2];
    
       
    % Grab the axis handle
    figure(fig_num);
    hold on;
    axis equal;
    grid on; grid minor;
    
    % Grab any data that may already be in the figure
    handles = get(fig_num,'UserData');
    
    % Associate the transform with this handle
    if isempty(vehicle_handle)
        ax = gca; % Grab current axes;
        transform_object = hgtransform('Parent',ax);
    else
        transform_object = hgtransform('Parent',vehicle_handle);
    end
    
    % Plot the tire
    handles.h_tires.(tire.name).body   = plot_box(0,0,tire_box,plot_str);  % Tire body    
    
    % Associate the plots with the transform object
    set(handles.h_tires.(tire.name).body,'Parent',transform_object);   
    
    % Rotate and translate the tire to current theta
    M = makehgtform('translate',[tire.position_x tire.position_y 0],'zrotate',tire.orientation_angle);
    set(transform_object,'Matrix',M);
    
    % Save the transform
    handles.h_tires.(tire.name).body_transform_object = transform_object;
    set(fig_num,'UserData',handles);
    
    
else % Update the line positions
    handles = get(fig_num,'UserData');
    M = makehgtform('translate',[tire.position_x tire.position_y 0],'zrotate',tire.orientation_angle);
    set(handles.h_tires.(tire.name).body_transform_object,'Matrix',M);
    if isnumeric(plot_str)
        plot_str = convertToColor(plot_str);
        set(handles.h_tires.(tire.name).body,'Color',plot_str);
    end   
    
    % Plot the tire usage?
    if ~isempty(tire.usage)
        if isnumeric(tire.usage)
            plot_str = convertToColor(tire.usage);
            set(handles.h_tires.(tire.name).body,'Color',plot_str);
        end
    end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    _____             _     _______ _          
%   / ____|           | |   |__   __(_)         
%  | (___  _ __   ___ | | _____| |   _ _ __ ___ 
%   \___ \| '_ \ / _ \| |/ / _ \ |  | | '__/ _ \
%   ____) | |_) | (_) |   <  __/ |  | | | |  __/
%  |_____/| .__/ \___/|_|\_\___|_|  |_|_|  \___|
%         | |                                   
%         |_|                                   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plotSpokeTire(tire, flag_make_new_plot,fig_num,plot_str,vehicle_handle) 

if flag_make_new_plot
    % Define the coordinates of the wheels (just a square for now). Start with
    % the front left side of the right wheel, and go clockwise. Then repeat for
    % the left wheel
    tire_radius = tire.length/2;
    tire_box = [...
        -tire.width/2  tire_radius;
        +tire.width/2  tire_radius;
        +tire.width/2 -tire_radius;
        -tire.width/2 -tire_radius];
   
    
    % Draw spokes?
    spoke_interval = 20*pi/180;
    tire_spoke_angles = (0:spoke_interval:2*pi)'; %  - tire.rolling_angle;
    N_spokes = length(tire_spoke_angles);
    tire_spoke_y = tire_radius*cos(tire_spoke_angles);
    tire_spoke_z = tire_radius*sin(tire_spoke_angles);
    
    % The spokes are set up as [x_start y_start x_end y_end]
    tire_spoke_start = ...
        [(-tire.width /2)*ones(N_spokes,1), tire_spoke_y, tire_spoke_z];
    tire_spoke_end = ...
        [(+tire.width /2)*ones(N_spokes,1), tire_spoke_y, tire_spoke_z];
    tire_spokes = [tire_spoke_start, tire_spoke_end];
       
    % Grab the axis handle
    figure(fig_num);
    hold on;
    axis equal;
    grid on; grid minor;
    
    % Grab any data that may already be in the figure
    handles = get(fig_num,'UserData');
    
    % Associate the transform with this handle
    if isempty(vehicle_handle)
        ax = gca; % Grab current axes;
        tire_body_transform_object = hgtransform('Parent',ax);
        tire_hub_transform_object = hgtransform('Parent',tire_body_transform_object);
        tire_spokes_transform_object = hgtransform('Parent',tire_hub_transform_object);
    else
        tire_body_transform_object = hgtransform('Parent',vehicle_handle);
        tire_hub_transform_object = hgtransform('Parent',tire_body_transform_object);
        tire_spokes_transform_object = hgtransform('Parent',tire_hub_transform_object);
    end
    
    % Plot the tire
    handles.h_tires.(tire.name).body   = plot_box(0,0,tire_box,plot_str);  % Tire body    
    
    % Associate the plots with the transform object
    set(handles.h_tires.(tire.name).body,'Parent',tire_body_transform_object);   
    
    % Plot the spokes   
    %     % Tag hidden angles to hide them
    %     spoke_angles = mod(spoke_angles,2*pi);
    %     spokes(spoke_angles>pi,:) = NaN;
    h = zeros(size(tire_spoke_angles));
    for i_spoke = 1:length(tire_spoke_angles(:,1))
        % h_spokes = plot_3Dspoke(flag_plot_exists, h_spokes_old, spoke, spoke_angle, varargin)    % plot all the spokes
        h(i_spoke)= plot_3Dspoke(0,0,tire_spokes(i_spoke,:),tire_spoke_angles(i_spoke),plot_str); % Tire spokes
        
        % Associate the plots with the transform object
        set(h(i_spoke),'Parent',tire_spokes_transform_object);
    end
    
    % Save the handles
    handles.h_tires.(tire.name).spokes = h;
    

    
    % Rotate and translate the tire body to current orientation angle (e.g.
    % like steering)
    M = makehgtform('translate',[tire.position_x tire.position_y 0],'zrotate',tire.orientation_angle);
    set(tire_body_transform_object,'Matrix',M);

    % Move the hub to the height above the body
    M = makehgtform('translate',[0 0 tire.position_z]);
    set(tire_hub_transform_object,'Matrix',M);
    
    % Rotate the spokes to current theta about the hub
    M = makehgtform('xrotate',-tire.rolling_angle);
    set(tire_spokes_transform_object,'Matrix',M);

    % Save the transforms
    handles.h_tires.(tire.name).body_transform_object = tire_body_transform_object;
    handles.h_tires.(tire.name).hub_transform_object = tire_hub_transform_object;
    handles.h_tires.(tire.name).spokes_transform_object = tire_spokes_transform_object;
    set(fig_num,'UserData',handles);
    
    
else % Update the line positions
    handles = get(fig_num,'UserData');
    
    % Move the body of the tire
    M = makehgtform('translate',[tire.position_x tire.position_y 0],'zrotate',tire.orientation_angle);
    set(handles.h_tires.(tire.name).body_transform_object,'Matrix',M);
    
    % Rotate the spokes around the hub
    M = makehgtform('xrotate',-tire.rolling_angle);
    set(handles.h_tires.(tire.name).spokes_transform_object,'Matrix',M);
    
        
    % Plot the tire usage?
    if ~isempty(tire.usage)
        if isnumeric(tire.usage)
            plot_str = convertToColor(tire.usage);
            set(handles.h_tires.(tire.name).body,'Color',plot_str);
        end
    end   
end

end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __           _  _______ _          
%  |  \/  |         | ||__   __(_)         
%  | \  / | ___  ___| |__ | |   _ _ __ ___ 
%  | |\/| |/ _ \/ __| '_ \| |  | | '__/ _ \
%  | |  | |  __/\__ \ | | | |  | | | |  __/
%  |_|  |_|\___||___/_| |_|_|  |_|_|  \___|
%                                          
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                         


function plotMeshTire(tire, flag_make_new_plot,fig_num,plot_str,vehicle_handle) 

if flag_make_new_plot
    % Define the coordinates of the wheels (just a square for now). Start with
    % the front left side of the right wheel, and go clockwise. Then repeat for
    % the left wheel
    tire_radius = tire.length/2;
    tire_box = [...
        -tire.width/2  tire_radius;
        +tire.width/2  tire_radius;
        +tire.width/2 -tire_radius;
        -tire.width/2 -tire_radius];
   
    
    % Draw dire
    width = tire.width;    
    t = [(0:0.1:1)*width];
    r = tire_radius*(sin(t*pi/width)).^0.03;
    [X,Y,Z] = cylinder(r,10);
    Z = Z*width - tire.width/2;
    plot_str = 0.1;
    % color_numbers = convertToColor(plot_str);    
    color_numbers = [0.4 0.4 0.4];

    CO(:,:,1) = ones(size(X))*color_numbers(1);
    CO(:,:,2) = ones(size(X))*color_numbers(2);
    CO(:,:,3) = ones(size(X))*color_numbers(3);
    h_1 = mesh(X,Y,Z,CO);
    set(h_1,'FaceColor','flat','FaceAlpha','0.8','EdgeColor',[0 0 0]);
    
    
    % Grab the axis handle
    figure(fig_num);
    hold on;
    axis equal;
    grid on; grid minor;
    
    % Grab any data that may already be in the figure
    handles = get(fig_num,'UserData');
    
    % Associate the transform with this handle
    if isempty(vehicle_handle)
        ax = gca; % Grab current axes;
        tire_body_transform_object = hgtransform('Parent',ax);
        tire_hub_transform_object = hgtransform('Parent',tire_body_transform_object);
        tire_spokes_transform_object = hgtransform('Parent',tire_hub_transform_object);
    else
        tire_body_transform_object = hgtransform('Parent',vehicle_handle);
        tire_hub_transform_object = hgtransform('Parent',tire_body_transform_object);
        tire_spokes_transform_object = hgtransform('Parent',tire_hub_transform_object);
    end
    
    % Plot the tire
    handles.h_tires.(tire.name).body   = plot_box(0,0,tire_box,plot_str);  % Tire body    
    
    % Associate the plots with the transform object
    set(handles.h_tires.(tire.name).body,'Parent',tire_body_transform_object);   
    
    % Associate the tire mesh with the tire_spoke_transform
    set(h_1,'Parent',tire_spokes_transform_object);
       
    % Save the handles
    handles.h_tires.(tire.name).spokes = h_1;
    

    
    % Rotate and translate the tire body to current orientation angle (e.g.
    % like steering)
    M = makehgtform('translate',[tire.position_x tire.position_y 0],'zrotate',tire.orientation_angle);
    set(tire_body_transform_object,'Matrix',M);

    % Move the hub to the height above the body and to correct orientation
    M = makehgtform('translate',[0 0 tire.position_z],'yrotate',pi/2);
    set(tire_hub_transform_object,'Matrix',M);
    
    % Rotate the spokes to current theta about the hub
    M = makehgtform('xrotate',-tire.rolling_angle);
    set(tire_spokes_transform_object,'Matrix',M);

    % Save the transforms
    handles.h_tires.(tire.name).body_transform_object = tire_body_transform_object;
    handles.h_tires.(tire.name).hub_transform_object = tire_hub_transform_object;
    handles.h_tires.(tire.name).spokes_transform_object = tire_spokes_transform_object;
    set(fig_num,'UserData',handles);
    
    
else % Update the line positions
    handles = get(fig_num,'UserData');
    
    % Move the body of the tire
    M = makehgtform('translate',[tire.position_x tire.position_y 0],'zrotate',tire.orientation_angle);
    set(handles.h_tires.(tire.name).body_transform_object,'Matrix',M);
    
    % Rotate the spokes around the hub
    M = makehgtform('zrotate',tire.rolling_angle);
    set(handles.h_tires.(tire.name).spokes_transform_object,'Matrix',M);
    
    % Plot the tire usage?
    if ~isempty(tire.usage)
        if isnumeric(tire.usage)
            plot_str = convertToColor(tire.usage);
            set(handles.h_tires.(tire.name).body,'Color',plot_str);
        end
    end  
end

end






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____  _       _   ____
%  |  __ \| |     | | |  _ \           
%  | |__) | | ___ | |_| |_) | _____  __
%  |  ___/| |/ _ \| __|  _ < / _ \ \/ /
%  | |    | | (_) | |_| |_) | (_) >  < 
%  |_|    |_|\___/ \__|____/ \___/_/\_\
%                                      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function h_box = plot_box(flag_plot_exists, h_box_old, corners, varargin)    % plot all the boxes

plot_str = 'k-';
plot_type = 1;

if 4 == nargin
    plot_str = varargin{1};
    if isnumeric(plot_str)
        plot_type = 2;
        plot_str = convertToColor(plot_str);
    end
end

% Close the box back into itself
xdata = [corners(:,1); corners(1,1)];
ydata = [corners(:,2); corners(1,2)];


% Check if plot already exists
if ~flag_plot_exists  % It does not exist, create it then
    if plot_type==1
        h_box = plot(xdata,ydata,plot_str);
    elseif plot_type==2
        h_box = plot(xdata,ydata,'Color',plot_str);
    end
else % It exists already
    set(h_box_old,'XData',xdata);
    set(h_box_old,'YData',ydata);
    if plot_type==2
        set(h_box_old,'Color',plot_str);
    end
    h_box = h_box_old;
end


end % Ends plot_box function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                 _ _______     _____      _            
%                                | |__   __|   / ____|    | |           
%    ___ ___  _ ____   _____ _ __| |_ | | ___ | |     ___ | | ___  _ __ 
%   / __/ _ \| '_ \ \ / / _ \ '__| __|| |/ _ \| |    / _ \| |/ _ \| '__|
%  | (_| (_) | | | \ V /  __/ |  | |_ | | (_) | |___| (_) | | (_) | |   
%   \___\___/|_| |_|\_/ \___|_|   \__||_|\___/ \_____\___/|_|\___/|_|   
%                                                                       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                                      


function plot_str = convertToColor(plot_str)
if length(plot_str(1,:))==1  % Is the number a scalar?
    color_map = hot(256); % jet(256);
    maxval = 120; % 256
    color_index = max(1,min(ceil(plot_str*maxval),256));
    plot_str = color_map(color_index,:);
end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____  _       _    _____             _
%  |  __ \| |     | |  / ____|           | |            
%  | |__) | | ___ | |_| (___  _ __   ___ | | _____  ___ 
%  |  ___/| |/ _ \| __|\___ \| '_ \ / _ \| |/ / _ \/ __|
%  | |    | | (_) | |_ ____) | |_) | (_) |   <  __/\__ \
%  |_|    |_|\___/ \__|_____/| .__/ \___/|_|\_\___||___/
%                            | |                        
%                            |_|                        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function h_spokes = plot_spokes(flag_plot_exists, h_spokes_old, spokes, spoke_angles, varargin)    %#ok<DEFNU> % plot all the spokes

plot_str = 'b-';
plot_type = 1;

if 5 == nargin
    plot_str = varargin{1};
    if isnumeric(plot_str)
        plot_type = 2;
        plot_str = convertToColor(plot_str);
    end
end

% % Tag hidden angles to hide them
% spoke_angles = mod(spoke_angles,2*pi);
% spokes(spoke_angles>pi,:) = NaN;


N_spokes = length(spokes(:,1));
xmatrix = [spokes(:,1), spokes(:,3), NaN*spokes(:,1)];

xdata = reshape(xmatrix',N_spokes*3,1);

ymatrix = [spokes(:,2), spokes(:,4), NaN*spokes(:,2)];
ydata = reshape(ymatrix',N_spokes*3,1);

% Check if plot already exists
if ~flag_plot_exists  % It does not exist, create it then
    if plot_type==1
        h_spokes = plot(xdata,ydata,plot_str);
    elseif plot_type==2
        h_spokes = plot(xdata,ydata,'Color',plot_str);
    end
else % It exists already
    set(h_spokes_old,'XData',xdata);
    set(h_spokes_old,'YData',ydata);
    if plot_type==2
        set(h_spokes_old,'Color',plot_str);
    end
    h_spokes = h_spokes_old;
end


end % Ends plot_spokes function



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         _       _   ____  _____   _____             _             
%        | |     | | |___ \|  __ \ / ____|           | |            
%   _ __ | | ___ | |_  __) | |  | | (___  _ __   ___ | | _____  ___ 
%  | '_ \| |/ _ \| __||__ <| |  | |\___ \| '_ \ / _ \| |/ / _ \/ __|
%  | |_) | | (_) | |_ ___) | |__| |____) | |_) | (_) |   <  __/\__ \
%  | .__/|_|\___/ \__|____/|_____/|_____/| .__/ \___/|_|\_\___||___/
%  | |                                   | |                        
%  |_|                                   |_|                        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function h_spokes = plot_3Dspoke(flag_plot_exists, h_spokes_old, spoke, spoke_angle, varargin)    % plot all the spokes

plot_str = 'b-';
plot_type = 1;

if 5 == nargin
    plot_str = varargin{1};
    if isnumeric(plot_str)
        plot_type = 2;
        plot_str = convertToColor(plot_str);
    end
end


% % Tag hidden angles to hide them
% spoke_angle = mod(spoke_angle,2*pi);
% spoke(spoke_angle>pi,:) = NaN;


xdata = [spoke(:,1), spoke(:,4), NaN*spoke(:,1)];
ydata = [spoke(:,2), spoke(:,5), NaN*spoke(:,2)];
zdata = [spoke(:,3), spoke(:,6), NaN*spoke(:,2)];


% Check if plot already exists
if ~flag_plot_exists  % It does not exist, create it then
    if plot_type==1
        h_spokes = plot3(xdata,ydata,zdata,plot_str);
    elseif plot_type==2
        h_spokes = plot3(xdata,ydata,zdata,'Color',plot_str);
    end
else % It exists already
    set(h_spokes_old,'XData',xdata);
    set(h_spokes_old,'YData',ydata);
    if plot_type==2
        set(h_spokes_old,'Color',plot_str);
    end
    h_spokes = h_spokes_old;
end


end % Ends plot_spokes function


