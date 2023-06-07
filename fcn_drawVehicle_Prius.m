function fcn_drawVehicle_Prius(vehicle,varargin)
% fcn_drawVehicle_Prius draws a vehicle shaped like a Prius.
% The user can specify a figure to use, or if none is specified, the
% current figure is used.
%
% FORMAT:
%
%          fcn_drawRobot(vehicle,varargin)
%
% INPUTS:
%          vehicle: a structure that contains the following fields:
%                   vehicle.yawAngle_radians - the yaw angle of the body of the vehicle
%                   [rad]
%                   vehicle.position_x - the x-position of the vehicle [m]
%                   vehicle.position_y - the y-position of the vehicle [m]
%                   vehicle.steeringAngle_radians - the steering angle [rad]
%
%                   vehicle.width - the width of the vehicle
%                   vehicle.length - the length of the vehicle
%                   vehicle.front_axle - the location of the front axle
%                   relative to the CG (the "a" parameter in the bicycle
%                   model, for example)
%                   vehicle.wheel_width - the width of the wheel
%                   vehicle.wheel_length - the length of the wheel
%                   vehicle.front_right_wheelangle - the radial angle of the front right wheel [rad]
%                   vehicle.front_left_wheelangle - the radial angle of the front left wheel [rad]
%                   vehicle.rear_right_wheelangle - the radial angle of the rear right wheel [rad]
%                   vehicle.rear_left_wheelangle - the radial angle of the rear leftt wheel [rad]
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
flag_show_tire = 1; % Flag to show the tires

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
if nargin < 1 || nargin > 2
    error('Incorrect number of input arguments.')
end

flag_make_new_plot = 0; % Default is not to make a new plot
if 2 == nargin
    temp = varargin{1};
    if ~isempty(temp)
        fig_num = temp;
        figure(fig_num);

        % Check to see if the figure has user data within it already
        handles = get(fig_num,'UserData');
        if isempty(handles)
            flag_make_new_plot = 1;
        end
    end
else
    fig = gcf; % create new figure with next default index
    fig_num = get(fig,'Number');
    flag_make_new_plot = 1;
end


%% fill in initial values
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _ _   _       ___      __   _
%  |_   _|     (_) | (_)     | \ \    / /  | |
%    | |  _ __  _| |_ _  __ _| |\ \  / /_ _| |_   _  ___  ___
%    | | | '_ \| | __| |/ _` | | \ \/ / _` | | | | |/ _ \/ __|
%   _| |_| | | | | |_| | (_| | |  \  / (_| | | |_| |  __/\__ \
%  |_____|_| |_|_|\__|_|\__,_|_|   \/ \__,_|_|\__,_|\___||___/
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tireNames = {'FrontLeft', 'FrontRight','RearRight', 'RearLeft'};

if flag_make_new_plot
    % Set up the tires with names
    for i_tire=1:length(vehicle.tire)
        vehicle.tire(i_tire).name = string(tireNames(i_tire)); % a string (no spaces) representing the tire's name
    end
    
    % Calculations to determine vehicle dimensions
    left_side  = -vehicle.width/2 + vehicle.tire(1).width/2;
    right_side =  vehicle.width/2 - vehicle.tire(1).width/2;
    front_side =  vehicle.front_axle;
    rear_side  =  vehicle.front_axle - vehicle.length;
    
    % Define the coordinates of the body (just a square for now). Start with
    % the front left and go clockwise around the vehicle
    body = [left_side front_side; right_side front_side; right_side rear_side; left_side rear_side];
    
    
    % Decorators
    [exterior_decorators, interior_decorators] = fcn_fillDecorators;
    % figure;
    % plot(exterior_decorators(:,1),exterior_decorators(:,2)); hold on;
    % axis equal;
    % plot(interior_decorators(:,1),interior_decorators(:,2));
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

 % Plot input results
    % This is done 2 ways, either the "new plot" way which plots the results
    % and stores the results into handles, or the "update" plot way which grabs
    % the handles and simply updates the position and color data.
    

if flag_make_new_plot
    % Rotate the vehicle to current theta
    offset = 90*pi/180;  % Use this to add offsets to the rotation matrix, for example if plotting is done in x-up situation
    
    % Fill in a rotation matrix
    R = [cos(vehicle.yawAngle_radians - offset) sin(vehicle.yawAngle_radians - offset);...
        -sin(vehicle.yawAngle_radians - offset)  cos(vehicle.yawAngle_radians - offset)];
    
    % Rotate the body
    body_rotated = body*R;
    exterior_decorators_rotated = exterior_decorators*R;
    interior_decorators_rotated = interior_decorators*R;
    
    
    % Translate the vehicle to current XY coordinates
    body_rotated_translated = body_rotated + [vehicle.position_x vehicle.position_y];
    exterior_decorators_rotated_translated = exterior_decorators_rotated + [vehicle.position_x vehicle.position_y];
    interior_decorators_rotated_translated = interior_decorators_rotated + [vehicle.position_x vehicle.position_y];
           
   
    
    
    figure(fig_num);
    hold on;
    axis equal;
    grid on; grid minor;
    
    
    % Create a transform object
    ax = gca; % Grab current axes;
    handles.transform_ground_to_body = hgtransform('Parent',ax);
    
    
    
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
    
else % Update the line positions
    % Retrieve the transform
    handles = get(fig_num,'UserData');
      
    % Move the body of the vehicle
    M = makehgtform('translate',[vehicle.position_x vehicle.position_y 0],'zrotate',vehicle.yawAngle_radians);
    set(handles.transform_ground_to_body,'Matrix',M);
    
    % Update tire usage or the tire rolling angles?
    for i_tire = 1:4
        % Grab the tire
        tire = vehicle.tire(i_tire);    
        tire.name = string(tireNames(i_tire)); % a string (no spaces) representing the tire's name
        % Steer the front tires?
        if (i_tire == 1)||(i_tire == 2)
            tire.orientation_angle = vehicle.steeringAngle_radians;
        end
        fcn_drawTire(tire,fig_num);
        
    end % Ends tire for loop
    
end % Ends flag if create new plot

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


function h_box = plot_box(flag_plot_exists, h_box_old, corners, varargin)    % plot all the boxes

plot_str = 'b-';
plot_type = 1;

if 4 == nargin
    plot_str = varargin{1};
    if isnumeric(plot_str)
        plot_type = 2;
    end
end

if 1==1
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
        h_box = h_box_old;
    end
else
    % Use a rectangle object? Not sure how to rotate it
end



end % Ends plot_box function

function [exterior_decorators, interior_decorators] = fcn_fillDecorators
% This fills in the "decorators" for the vehicle

% imshow('toyota-prius-2010_resized_to_1pixel_per_cm.bmp','InitialMagnification',100)

%%
body_outline_x = [
    9.1958
    9.1958
    9.1958
    8.6688
    10.2498
    11.3038
    12.3579
    14.9930
    19.2091
    24.4792
    27.6413
    31.8575
    37.1276
    45.5599
    55.0462
    65.0595
    80.8700
    124.0854
    137.7878
    160.4495
    178.8951
    194.1785
    222.6374
    242.1370
    261.6366
    289.5685
    309.5951
    331.7298
    352.8105
    380.7424
    393.3908
    404.9851
    414.4714
    423.4307
    431.3359
    434.4980
    436.0791
    438.1872
    441.8763
    443.4573
    446.0924
    447.6735
    447.6735
    449.2545
    450.3085
    451.3626
    451.3626
    451.3626
    ];

body_outline_y = [
    108.4409
    102.1167
    94.7384
    88.9413
    79.9820
    72.0767
    63.6445
    55.2122
    47.3070
    42.0368
    37.2937
    31.4965
    28.8614
    27.2803
    24.1182
    22.5372
    22.5372
    22.0102
    22.5372
    22.5372
    22.5372
    22.5372
    21.4832
    22.5372
    22.5372
    22.5372
    22.5372
    23.0642
    23.0642
    22.5372
    23.0642
    23.5912
    25.6993
    28.3344
    29.9154
    30.4424
    34.1316
    37.8207
    43.0908
    49.4150
    54.6852
    66.8066
    73.1308
    76.2929
    85.2522
    94.7384
    104.2247
    113.1840];

hood_x = [
    11.3038
    11.3038
    11.8309
    13.4119
    13.9389
    14.9930
    15.5200
    16.5740
    18.6821
    18.6821
    25.0063
    32.3845
    31.3305
    30.2764
    28.6954
    28.1684
    27.1143
    26.5873
    25.5333
    25.0063
    24.4792
    24.4792];


hood_y = [
    108.9679
    105.2788
    101.0626
    100.0086
    95.2655
    90.5223
    83.6711
    78.4009
    71.0227
    69.9687
    67.8606
    64.1715
    71.5497
    76.8199
    81.0360
    85.2522
    88.9413
    92.1034
    95.7925
    97.9005
    101.0626
    108.9679];

headlight_x =[
    24.4792
    27.1143
    31.3305
    35.0196
    40.2897
    46.6139
    52.9381
    59.7894
    64.0055
    67.6946
    73.4918
    77.1809
    73.4918
    67.6946
    60.3164
    56.1002
    57.6813
    58.7353
    56.1002
    51.8841
    47.1410
    41.8708
    35.5466
    30.8034
    25.0063];


headlight_y = [
    60.4824
    53.6312
    47.8340
    42.0368
    36.7666
    32.5505
    29.9154
    29.3884
    29.3884
    29.9154
    31.4965
    32.0235
    34.6586
    36.2396
    37.8207
    39.4017
    40.9828
    43.0908
    45.1989
    48.3610
    52.0501
    54.1582
    56.7933
    58.9013
    60.4824];

mirror_x =[
    158.3414
    152.0172
    150.4362
    148.8551
    150.4362
    152.0172
    154.6523
    159.3955
    160.9765
    164.6656
    165.1926
    163.6116
    163.6116
    162.5576
    160.9765
    159.9225
    158.3414];


mirror_y = [
    27.8074
    29.3884
    29.9154
    25.1723
    20.9561
    16.7400
    11.9969
    9.3618
    7.7807
    7.7807
    10.4158
    12.5239
    16.2130
    19.3751
    23.5912
    25.1723
    27.8074];

little_front_window_side_door_x = [
    116.4448
    127.0948
    140.2506
    150.9005
    156.5388
    160.2976
    154.0329
    145.8888
    135.8653
    116.4448];


little_front_window_side_door_y = [
    31.2255
    30.5991
    30.5991
    30.5991
    31.2255
    38.7431
    38.1167
    36.8637
    35.6108
    31.2255];



windshield_x = [
    93.8919
    93.2655
    93.8919
    95.1449
    96.3978
    97.6507
    100.1566
    101.4096
    103.9154
    106.4213
    111.4330
    118.9507
    127.0948
    133.3594
    144.0094
    153.4064
    166.5623
    176.5857
    186.6092
    195.3798
    196.0063
    194.7533
    193.5004
    194.1269
    192.2475
    192.8739
    192.8739
    192.8739];

windshield_y = [
    111.4135
    103.2694
    96.3782
    94.4988
    81.9695
    70.6930
    60.0431
    50.6460
    43.1284
    38.7431
    38.7431
    39.3696
    39.9961
    40.6226
    41.8755
    43.7549
    44.3814
    46.2608
    47.5137
    50.0196
    55.6578
    60.0431
    65.6813
    80.7165
    86.9812
    95.1253
    106.4017
    110.1605];

large_front_window_side_door_x = [
    160.9240
    170.9475
    185.9828
    215.4268
    235.4738
    244.8708
    251.1355
    236.7267
    222.3179
    217.9327
    206.0298
    192.8739
    179.0916
    165.3093
    160.9240];

large_front_window_side_door_y = [
    32.4785
    31.8520
    32.4785
    32.4785
    32.4785
    32.4785
    46.8872
    46.8872
    46.8872
    46.8872
    45.0078
    43.1284
    41.2490
    39.9961
    31.8520];

large_rear_window_side_door_x = [
    253.6413
    263.6648
    279.9530
    295.6147
    313.7823
    331.9499
    338.8410
    325.6852
    312.5294
    263.6648
    259.2796];


large_rear_window_side_door_y = [
    33.7314
    33.7314
    34.3579
    34.3579
    36.2373
    36.2373
    37.4902
    46.8872
    47.5137
    48.1402
    34.9843];

small_rear_window_side_door_x = [
    343.2263
    349.4910
    357.6351
    363.8998
    367.6586
    368.9115
    365.7792
    356.3821
    353.8763
    341.3469
    332.5764
    343.2263];


small_rear_window_side_door_y = [
    37.4902
    37.4902
    36.8637
    38.7431
    39.9961
    42.5020
    44.3814
    46.2608
    46.2608
    47.5137
    48.1402
    37.4902];


rear_window_x = [
    358.2616
    357.6351
    358.8880
    358.8880
    357.6351
    357.6351
    357.0086
    367.6586
    380.1879
    387.7056
    397.7291
    403.9937
    415.8966
    416.5231
    417.7760
    419.0290
    419.6554
    420.2819
    420.2819];


rear_window_y = [
    109.5341
    102.6429
    90.1135
    80.0901
    71.3195
    57.5372
    54.4049
    55.0313
    54.4049
    53.1519
    53.1519
    52.5255
    52.5255
    63.1754
    71.3195
    81.3430
    89.4871
    100.7635
    109.5341];

rear_spoiler_x = [
    416.5231
    421.5348
    425.9201
    430.3054
    432.8113
    434.0642
    436.5701
    437.8230
    438.4495
    438.4495
    439.7024
    440.3289
    440.9554
    440.9554
    440.9554
    442.2083];

rear_spoiler_y = [
    51.2725
    51.2725
    51.2725
    51.2725
    52.5255
    55.6578
    59.4166
    63.8019
    65.0548
    67.5607
    73.1989
    78.8371
    81.9695
    90.7400
    100.7635
    108.2811];

taillight_x =[
    407.1261
    412.7643
    416.5231
    425.2937
    428.4260
    430.3054
    433.4377
    434.0642
    434.0642
    432.1848
    429.0525
    427.1731
    425.9201
    425.2937
    425.2937
    425.2937
    427.1731
    428.4260
    429.0525
    432.1848
    433.4377
    435.3171
    431.5583
    428.4260
    425.9201
    425.2937
    418.4025
    414.0172
    410.2584
    407.1261
    405.8731
    405.8731];


taillight_y = [
    36.2373
    34.3579
    34.3579
    33.7314
    36.2373
    39.3696
    43.1284
    48.7666
    52.5255
    52.5255
    51.8990
    47.5137
    45.0078
    42.5020
    40.6226
    38.1167
    36.8637
    36.2373
    36.8637
    41.2490
    44.3814
    52.5255
    53.1519
    52.5255
    51.8990
    48.7666
    45.0078
    42.5020
    40.6226
    38.1167
    36.8637
    36.8637];

%% Interior decorators

interior_perimeter_x = [
    76.9773
    76.9773
    76.9773
    78.8567
    81.3626
    85.1214
    87.6273
    92.0125
    98.2772
    103.2890
    110.8066
    118.3242
    142.1300
    172.2005
    200.3915
    224.1973
    252.3884
    281.8324
    306.8912
    331.3234
    343.2263
    351.9969
    360.7674
    376.4291
    389.5850
    398.3555
    407.1261
    413.3908
    422.7878
    433.4377
    439.7024
    443.4612
    444.0877
    444.0877
    445.3406];


interior_perimeter_y =[
    109.5341
    103.2694
    94.4988
    86.3547
    76.9577
    68.1872
    62.5489
    56.9107
    45.6343
    39.3696
    31.8520
    29.9726
    29.9726
    28.7197
    29.3461
    29.3461
    29.3461
    29.3461
    27.4667
    27.4667
    28.0932
    29.3461
    31.2255
    32.4785
    33.7314
    34.9843
    36.8637
    39.9961
    46.8872
    50.6460
    56.2843
    65.0548
    75.7048
    86.9812
    107.0282];


interior_frontseat_x = [
    187.2357
    185.9828
    185.3563
    185.3563
    185.3563
    182.8504
    183.4769
    183.4769
    183.4769
    185.3563
    185.3563
    185.3563
    185.9828
    189.1151
    196.0063
    202.2709
    211.6680
    221.0650
    224.8238
    224.8238
    229.2091
    231.0885
    231.0885
    231.7150
    232.3414
    232.3414
    231.7150
    229.8356
    226.7032
    221.6915
    218.5591
    214.8003
    211.0415
    206.6562
    201.6445
    195.3798
    193.5004
    191.6210
    188.4886
    291.8559
    224.8238
    231.0885
    239.2326
    246.7502
    253.0149
    258.0266
    258.6531
    258.6531
    259.2796
    259.2796
    257.4002
    256.1472
    254.2678
    248.0031
    242.3649
    236.7267
    228.5826
    224.8238
    225.4503
    332.5764
    242.9914
    247.3767
    249.2561
    254.8943
    256.1472
    256.7737
    256.1472
    256.7737
    256.1472
    254.8943
    254.2678
    243.6179
    242.9914];


interior_frontseat_y = [
    98.8841
    97.6312
    95.7518
    92.6194
    89.4871
    88.2341
    76.9577
    65.6813
    63.8019
    63.1754
    60.0431
    55.0313
    51.8990
    50.0196
    49.3931
    49.3931
    49.3931
    48.7666
    50.6460
    53.1519
    55.0313
    58.1637
    62.5489
    67.5607
    75.0783
    84.4753
    87.6077
    91.9930
    94.4988
    97.0047
    97.6312
    98.2576
    100.1370
    99.5106
    99.5106
    100.1370
    100.1370
    98.8841
    98.2576
    224.8042
    47.5137
    47.5137
    47.5137
    49.3931
    51.8990
    58.7901
    65.0548
    71.9460
    80.7165
    86.9812
    90.1135
    93.8724
    97.0047
    98.2576
    99.5106
    99.5106
    99.5106
    99.5106
    97.6312
    212.2749
    62.5489
    62.5489
    62.5489
    64.4283
    68.8136
    71.3195
    74.4518
    78.8371
    81.3430
    83.2224
    84.4753
    85.1018
    62.5489];
% Set the reset points to zero on this one capture
interior_frontseat_x(interior_frontseat_y>200) = NaN;
interior_frontseat_y(interior_frontseat_y>200) = NaN;


interior_rearseat_x = [
    278.0736
    278.7001
    275.5677
    277.4471
    278.7001
    279.3265
    283.7118
    288.0971
    325.6852
    327.5646
    321.2999
    321.2999
    320.6735
    323.8058
    324.4323
    324.4323
    320.6735
    321.2999
    321.9264
    449.0995
    321.9264
    350.7439
    357.0086
    355.7557
    344.4792
    341.3469
    340.7204
    342.5998
    350.1175
    353.8763
    355.7557
    356.3821
    355.7557
    466.0141  %
    330.6969
    341.9734
    350.7439
    355.1292
    355.7557
    470.3994
    350.1175
    350.1175
    347.6116
    338.8410
    335.7087
    335.7087];

interior_rearseat_y = [
    110.1605
    93.8724
    91.3665
    61.9225
    60.0431
    55.0313
    50.0196
    44.3814
    44.3814
    48.1402
    50.6460
    52.5255
    54.4049
    56.9107
    90.7400
    92.6194
    93.8724
    97.6312
    108.9076
    284.9452
    98.2576
    98.2576
    97.6312
    85.7283
    85.7283
    86.3547
    65.6813
    64.4283
    64.4283
    64.4283
    66.3078
    81.9695
    85.7283
    263.6453 %
    46.8872
    46.8872
    46.2608
    48.1402
    65.0548
    236.0807
    111.4135
    100.7635
    99.5106
    99.5106
    100.1370
    109.5341];
% Set the reset points to zero on this one capture
interior_rearseat_x(interior_rearseat_y>200) = NaN;
interior_rearseat_y(interior_rearseat_y>200) = NaN;

%%

% Fix the points to be flipped. The centerline for the above data is at y =
% 100 cm.
offset_x = 97+140; % Units are cm. Measured 97 cm in figure to front tire, which is assumed to be 140 cm in front of CG
offset_y = 110; % Units are cm.
[body_outline_x, body_outline_y] = fcn_FlipAndFix(offset_x, offset_y,body_outline_x,body_outline_y);
[hood_x, hood_y] = fcn_FlipAndFix(offset_x, offset_y,hood_x,hood_y);
[headlight_x, headlight_y] = fcn_FlipAndFix(offset_x, offset_y,headlight_x,headlight_y);
[mirror_x, mirror_y] = fcn_FlipAndFix(offset_x, offset_y,mirror_x,mirror_y);
[windshield_x, windshield_y] = fcn_FlipAndFix(offset_x, offset_y,windshield_x,windshield_y);
[little_front_window_side_door_x, little_front_window_side_door_y] = fcn_FlipAndFix(offset_x, offset_y,little_front_window_side_door_x,little_front_window_side_door_y);
[large_front_window_side_door_x, large_front_window_side_door_y] = fcn_FlipAndFix(offset_x, offset_y,large_front_window_side_door_x,large_front_window_side_door_y);
[large_rear_window_side_door_x, large_rear_window_side_door_y] = fcn_FlipAndFix(offset_x, offset_y,large_rear_window_side_door_x,large_rear_window_side_door_y);
[small_rear_window_side_door_x, small_rear_window_side_door_y] = fcn_FlipAndFix(offset_x, offset_y,small_rear_window_side_door_x,small_rear_window_side_door_y);
[rear_window_x, rear_window_y] = fcn_FlipAndFix(offset_x, offset_y,rear_window_x,rear_window_y);
[rear_spoiler_x, rear_spoiler_y] = fcn_FlipAndFix(offset_x, offset_y,rear_spoiler_x,rear_spoiler_y);
[taillight_x, taillight_y] = fcn_FlipAndFix(offset_x, offset_y,taillight_x,taillight_y);

% Interior items
[interior_perimeter_x, interior_perimeter_y] = fcn_FlipAndFix(offset_x, offset_y,interior_perimeter_x,interior_perimeter_y);
[interior_frontseat_x, interior_frontseat_y] = fcn_FlipAndFix(offset_x, offset_y,interior_frontseat_x,interior_frontseat_y);
[interior_rearseat_x, interior_rearseat_y] = fcn_FlipAndFix(offset_x, offset_y,interior_rearseat_x, interior_rearseat_y);

% For debugging
if 1==0
    figure(3);
    clf;
    plot(body_outline_x,body_outline_y,'k');
    hold on;
    axis equal;
    xlim([-500 500]);
    plot(hood_x,hood_y,'k');
    plot(headlight_x,headlight_y,'k');
    %plot(top_hood_x,top_hood_y,'k');
    plot(mirror_x,mirror_y,'k');
    plot(windshield_x,windshield_y,'k');
    plot(little_front_window_side_door_x,little_front_window_side_door_y,'k');
    plot(large_front_window_side_door_x,large_front_window_side_door_y,'k');
    plot(large_rear_window_side_door_x,large_rear_window_side_door_y,'k');
    plot(small_rear_window_side_door_x,small_rear_window_side_door_y,'k');
    plot(rear_window_x,rear_window_y,'k');
    plot(rear_spoiler_x,rear_spoiler_y,'k');
    plot(taillight_x,taillight_y,'k');
    
    
    plot(interior_perimeter_x, interior_perimeter_y,'-','Color',[0.5 0.5 0.5]);
    plot(interior_frontseat_x, interior_frontseat_y,'-','Color',[0.5 0.5 0.5]);
    plot(interior_rearseat_x, interior_rearseat_y,'-','Color',[0.5 0.5 0.5]);
end

% Save results, scale and rotate them too
exterior_decorators = 0.01*[...
    body_outline_x,body_outline_y;
    NaN, NaN;
    hood_x,hood_y;
    NaN, NaN;
    headlight_x,headlight_y;
    NaN, NaN;
    mirror_x,mirror_y;
    NaN, NaN;
    windshield_x,windshield_y;
    NaN, NaN;
    little_front_window_side_door_x,little_front_window_side_door_y;
    NaN, NaN;
    large_front_window_side_door_x,large_front_window_side_door_y;
    NaN, NaN;
    large_rear_window_side_door_x,large_rear_window_side_door_y;
    NaN, NaN;
    small_rear_window_side_door_x,small_rear_window_side_door_y;
    NaN, NaN;
    rear_window_x,rear_window_y;
    NaN, NaN;
    rear_spoiler_x,rear_spoiler_y;
    NaN, NaN;
    taillight_x,taillight_y]*[0 -1; 1 0];

interior_decorators = 0.01*[...
    interior_perimeter_x, interior_perimeter_y;
    NaN, NaN;
    interior_frontseat_x, interior_frontseat_y;
    NaN, NaN;
    interior_rearseat_x, interior_rearseat_y
    ]*[0 -1; 1 0];
end


function [body_outline_x_new, body_outline_y_new] = fcn_FlipAndFix(offset_x, offset_y,body_outline_x,body_outline_y)
body_outline_x_fixed = body_outline_x - offset_x;
body_outline_y_fixed = body_outline_y - offset_y;

other_side_x =  flipud(body_outline_x_fixed);
other_side_y = -flipud(body_outline_y_fixed);

% Check to see if the data cross over the centerline. We do this by
% checking if any points are within some limit of the centerline
flag_connect_top_to_bottom = 0; % Set the default value
threshold = 5; % 5 cm
if any(body_outline_y_fixed.^2 < threshold.^2)
    flag_connect_top_to_bottom = 1;
end

if flag_connect_top_to_bottom
    body_outline_x_new = [body_outline_x_fixed;other_side_x; body_outline_x_fixed(1)];
    body_outline_y_new = [body_outline_y_fixed;other_side_y;  body_outline_y_fixed(1)];
else % Use an NaN value to disconnect one side from the other
    body_outline_x_new = [body_outline_x_fixed;NaN; other_side_x];
    body_outline_y_new = [body_outline_y_fixed; NaN; other_side_y];
end



end



function plot_str = convertToColor(plot_str)
if length(plot_str(1,:))==1  % Is the number a scalar?
    color_map = hot(256); % jet(256);
    maxval = 120; % 256
    color_index = max(1,min(ceil(plot_str*maxval),256));
    plot_str = color_map(color_index,:);
end
end