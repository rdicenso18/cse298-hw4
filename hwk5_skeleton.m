%===============================================================
function hwk5_skeleton
%===============================================================
% Note that it is considered bad practice to use global variables when
% programming.  Don't do this at home!!!
global V_ROBOT OMEGA_MAX DT_ODOM WHEELBASE SIGMA_WHEEL SIGMA_BEARING;

% Initialize the robot parameters
V_ROBOT=5;          % Robot velocity in m/s
DT_ODOM = 0.1;      % Update rate for odometry (10 Hz)    
OMEGA_MAX=pi/4;     % Angular velocity bound in radians/s 
WHEELBASE = 1.0;    % Wheelbase of the robot in meters
SIGMA_WHEEL = 0.1;  % This is the std dev of the wheel velocity in m/s

% Camera measurement parameters
SIGMA_BEARING = 3*pi/180;   % Std dev for bearing estimates in radians

% Set up the figure 
close all; figure; axis equal; axis([0 120 0 80]); hold on;

% Initial robot pose
x0 = 15;  y0 = 15;  theta0 = 0;

% This is the actual robot position.  The robot is green.
robot = make_robot( x0, y0, theta0, 'size', 1.5, 'color', 'g', 'make_trail', 1 );
% This is the position of the robot as estimated by your EKF.  It is redd
robot_hat = make_robot( x0+10*randn, y0+10*randn, theta0+pi/8*randn, 'size', 1.5, 'color', 'r', 'make_trail', 1 );
% Make the path for the robot to follow
rectangle( 'position', [15 15 90 50],'linestyle',':', 'edgecolor', 'k' );

% This is our initial Covariance estimate.  Note is is NOT correct.  You
% can play with this as you see fit based upon other parameters given. 
P = [100 0 0; 0 100 0; 0 0 (pi/18)^2];

num_cameras = 0;
button = 1;
while 1    
    % Gets a single mouse input from the user.  The corresponding x-y
    % position will be the location of the camera
    [ x, y, button ] = ginput(1);
    % If you hit the right mouse button, the loop will terminate
    if button==3, break; end
    num_cameras = num_cameras+1;
    % This is the position of the camera, the range it is capable of 
    % transmitting and the color it is initially set to.  
    camera(num_cameras) = make_camera( x, y, 20, [0.5 0.5 0.5] );
end

% This is just here for demonstration purposes showing how the robots will
% move.  You will need to comment out this loop, and comment in the while
% loop below where you will add the EKF code.  
for i=1:300
    % leg 1
    if (i <= 90)
        robot = move_robot( robot, robot.x + 1, robot.y, robot.theta);
        
    % leg 2
    elseif (i > 90 && i <= 95)
        robot = move_robot( robot, robot.x, robot.y, robot.theta + (pi/2)/5);
    
    % leg 3
    elseif (i > 95 && i <= 145)
        robot = move_robot( robot, robot.x, robot.y+1, robot.theta);
    
    % leg 4
    elseif (i > 145 && i <= 150)
        robot = move_robot( robot, robot.x, robot.y, robot.theta + (pi/2)/5);
    
    % leg 5
    elseif (i > 150 && i <= 240)
        robot = move_robot( robot, robot.x - 1, robot.y, robot.theta);
    
    % leg 6
    elseif (i > 240 && i <= 245)
        robot = move_robot( robot, robot.x, robot.y, robot.theta + (pi/2)/5);
        
    % leg 7
    elseif (i > 245 && i <= 295)
        robot = move_robot( robot, robot.x, robot.y-1, robot.theta);
    
    % leg 8
    else
        robot = move_robot( robot, robot.x, robot.y, robot.theta + (pi/2)/5);
    end
    
    %robot = move_robot( robot, robot.x+cos(robot.theta), robot.y+sin(robot.theta), robot.theta+0.05*randn );
    %robot_hat = move_robot( robot_hat, robot_hat.x+cos(robot_hat.theta), robot_hat.y+sin(robot_hat.theta), robot_hat.theta+0.05*randn );
    pause(0.05);
end


% %******************************************
% % ADD OTHER CODE HERE
% %******************************************
 current_leg=1;
 while current_leg~=9
     % Measurement Update Phase
     for i=1:num_cameras
         % Each camera is tested to see if it can be seen by the robot.  
         % Visible cameras are set to green, and a line reflecting its
         % measured position as estimated by the robot is also plotted.  
         [ camera(i), bearing ] = test_camera( camera(i), robot );        
         if ~isempty( bearing )
             [ robot_hat, P ] = MeasurementUpdate( robot_hat, P, camera(i), bearing );
 
% %******************************************
% % ADD OTHER CODE HERE
% %******************************************
% 
         end
     end
 
     % Time Update Phase.  Note we do multiple time updates for each
     % measurement update because the update rate of the odometry is higher
     for j=1:1/DT_ODOM
         [ robot, robot_hat, P ] = TimeUpdate( robot, robot_hat, P, legV, legOmega );        
% %******************************************
% % ADD OTHER CODE HERE
% %******************************************
     end
 
 end

%==================================================================================
function [ robot, robot_hat, P ] = TimeUpdate( robot, robot_hat, P, v, omega )
%==================================================================================
global DT_ODOM WHEELBASE SIGMA_WHEEL;
%******************************************
% ADD OTHER CODE HERE
%******************************************


%==================================================================================
function [ robotHat, P ] = MeasurementUpdate( robot_hat, P, camera, range )
%==================================================================================
global SIGMA_BEARING;
%******************************************
% for i=1:camera.size
 %       if (range <= camera(i).range)
            
%******************************************

%==================================================================================
function camera = make_camera(x, y, range, color)
%==================================================================================
    % Initialize the camera
    camera.x = x;
    camera.y = y;
    camera.range = range;
    camera.color = color;
    
    % Camera patch
    fig_coords = [x-1, x+1, x; y, y, y+2];
    camera.fig_coords = fig_coords;
    camera.h = patch(fig_coords(1,:), fig_coords(2,:), color);
