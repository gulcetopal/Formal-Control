%04.09.2020
close all;


path = [10.25   00.00;
        10.25   60.00;
        40.75   80.75;
        85.25   80.75;
        110.45  60.00;
        60.45  60.00];
    
% waypoints manual
waypoints = [path; path(end,:)];
priorWaypoint      = waypoints(1,:);
activeWaypoint     = waypoints(2,:);

% initialization
initialLocation    = path(1,:);
initialOrientation = pi/2;
robotCurrentPose   = [initialLocation initialOrientation];
pathGoal           = path(end,:);

% constants
lookaheadDistance = 1;
desiredVelocity   = 1;

% controller
controller                       = controllerPurePursuit;
controller.DesiredLinearVelocity = desiredVelocity;
controller.MaxAngularVelocity    = 2;
controller.LookaheadDistance     = lookaheadDistance;
controller.Waypoints             = path;

% data
odomHistory          = zeros(6000,3);
lateralTrackingError = zeros(6000,4);
curvatureHistory     = zeros(6000,1);
arctanHistory        = zeros(6000,1);

% setup
goalRadius = 2;
i = 2; k = 3;
distanceToGoal = norm(pathGoal - robotCurrentPose(1:2));
sampleTime = 0.1; time = 0; flag = 0; w = 0; theta = 0;

figure;
plot(path(:,1), path(:,2), 'k--d')
title('Simulation')
hold on;
while (distanceToGoal > goalRadius)
    
    % controller parameters
    [~, ~, laPoint] = controller(robotCurrentPose(1,:));
    theta = angleWrap(robotCurrentPose(3));
    x = angleWrap(atan((laPoint(2) - robotCurrentPose(2))/(laPoint(1) - robotCurrentPose(1)))); arctanHistory(i) = x;
%     disp([rad2deg(arctanHistory(i)), rad2deg(arctanHistory(i-1))])
    x = headingFilter(arctanHistory(i), arctanHistory(i-1)); arctanHistory(i) = x;  
%     disp(rad2deg(x))
    alpha = angleWrap(x - theta);
%     disp([rad2deg(x) rad2deg(theta) rad2deg(alpha)])
    R = lookaheadDistance/(2*sin(alpha));
    
    % kinematics
    v = desiredVelocity;
    w = v/R;
    vel = [v*cos(robotCurrentPose(3)) v*sin(robotCurrentPose(3)) w];

    lateralTrackingError(i,4) = lookaheadDistance*sin(alpha);
    
    % robot pose update
    robotCurrentPose = robotCurrentPose + vel*sampleTime;
    odomHistory(i,:) = robotCurrentPose;

    
    % lateral tracking error
    priorWaypointToOdom = robotCurrentPose(1:2) - waypoints(k-2,:);
    directionVector     = waypoints(k-1,:) - waypoints(k-2,:);
    crossProduct = cross([priorWaypointToOdom 0], [directionVector 0]);
    lateralTrackingError(i,1) = norm(crossProduct) / norm(directionVector) * sign(crossProduct(3));
    alpha1 = asind(lateralTrackingError(i,1)/norm(priorWaypointToOdom));
    
    priorWaypointToOdom = robotCurrentPose(1:2) - waypoints(k-1,:);
    directionVector     = waypoints(k,:) - waypoints(k-1,:);
    crossProduct = cross([priorWaypointToOdom 0], [directionVector 0]);
    lateralTrackingError(i,2) = norm(crossProduct) / norm(directionVector) * sign(crossProduct(3));
    alpha2 = asind(lateralTrackingError(i,2)/norm(priorWaypointToOdom));
    
    if (abs(lateralTrackingError(i,1)) < abs(lateralTrackingError(i,2)) )
        lateralTrackingError(i,3) = lateralTrackingError(i,1);
    else
        lateralTrackingError(i,3) = lateralTrackingError(i,2);
    end
    
    distanceToWaypoint = norm(activeWaypoint - robotCurrentPose(1:2));
    
    % waypoint update
    if (abs(lateralTrackingError(i,2)/lateralTrackingError(i,1)) < 1 || distanceToWaypoint < 0.05)
        priorWaypoint = activeWaypoint;
        activeWaypoint = waypoints(k,:);
        disp("Waypoint Passed")
        plot(robotCurrentPose(1), robotCurrentPose(2), 'c*', 'MarkerSize', 4, 'LineWidth',2)
        k = k + 1;
    end
    
    
    % DEBUG
    plot(robotCurrentPose(1), robotCurrentPose(2), 'bo', 'MarkerSize', 2)
    plot(laPoint(1), laPoint(2), 'r*', 'MarkerSize', 2)
%     pause;
    
    % loop update
    i = i + 1;
    time = time + sampleTime;
    distanceToGoal = norm(pathGoal - robotCurrentPose(1:2));
    
end
hold off;
disp("Goal Reached")
totalError = [trapz(abs(lateralTrackingError(:,3))) trapz(abs(lateralTrackingError(:,4)))]
timeInterval = 0:0.1:time;
ind = int64(time/sampleTime);
str = "LA Distance: " + num2str(controller.LookaheadDistance) + "  Speed: " + num2str(controller.DesiredLinearVelocity);

figure; 
plot(timeInterval, lateralTrackingError(1:ind,3), 'b', timeInterval, lateralTrackingError(1:ind,4), 'g')
title('Lateral Tracking Error')
grid on; 
figure; grid on;
plot(path(:,1), path(:,2), 'k--d', odomHistory(1:ind,1), odomHistory(1:i-1,2), 'r')
title(str)

% plot(odomHistory(1:i-1), timeInterval, 'r')
% sorunlu path
path = [-10.25   00.00;
        -10.25   60.00;
        -40.75   -80.75;
        -85.25   -80.75;
        -110.45  -60.00;
        -130.45  -75.00];
    
% klasik path
path = [10.25   00.00;
        10.25   60.00;
        40.75   80.75;
        85.25   80.75;
        110.45  60.00;
        110.45  00.00];
    
%
path = [-10.25   00.00;
        -20.25   60.00;
        -60.75   -80.75;
        -85.25   -80.75;
        -110.45  -60.00;
        -130.45  -75.00];
   
