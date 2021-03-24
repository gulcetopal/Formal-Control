close all;

path = [10.25   00.00;
        10.25   60.00;
        40.75   80.75;
        85.25   80.75;
        110.45  60.00;
        110.45  00.00];
    
% waypoints manual
waypoints = [path zeros(length(path),1)];
priorWaypoint      = waypoints(1,:);
activeWaypoint     = waypoints(2,:);
distanceToWaypoint = 3;

% initialization
initialLocation    = path(1,:);
initialOrientation = pi/2;
robotCurrentPose   = [initialLocation initialOrientation];
pathGoal           = path(end,:);

% controller
controller                       = controllerPurePursuit;
controller.DesiredLinearVelocity = 2;
controller.MaxAngularVelocity    = 2;
controller.LookaheadDistance     = 0.2;
controller.Waypoints             = path;


% data
odomHistory          = zeros(2000,3);
lateralTrackingError = zeros(2000,1);
curvatureHistory     = zeros(2000,1);

% setup
goalRadius = 2;
i = 1; k = 3;
distanceToGoal = norm(pathGoal - robotCurrentPose(1:2));
sampleTime = 0.1; time = 0;

figure;
plot(path(:,1), path(:,2), 'k--d')
hold on;
while (distanceToGoal > goalRadius)
    
    % obtain controller output
    [v w laPoint] = controller(robotCurrentPose(1,:));
    
    % kinematics
    vel = [v*cos(robotCurrentPose(3)) v*sin(robotCurrentPose(3)) w];
    
    % robot pose update
    robotCurrentPose = robotCurrentPose + vel*sampleTime;
    odomHistory(i,:) = robotCurrentPose;
    
    % vector definitions
    priorWaypointToOdom = robotCurrentPose - priorWaypoint;
    directionVector     = activeWaypoint - priorWaypoint;
    
    % lateral tracking error
    crossProduct = cross(priorWaypointToOdom, directionVector);
    lateralTrackingError(i) = norm(crossProduct) / norm(directionVector) * sign(crossProduct(3));
%     disp(lateralTrackingError(i))
    
    % waypoint update
    distanceToWaypoint = norm(activeWaypoint(1:2) - robotCurrentPose(1:2));
    if (distanceToWaypoint < goalRadius && k <= length(path))
       priorWaypoint = activeWaypoint;
       activeWaypoint = waypoints(k,:);
       k = k + 1;
    end
    
    i = i + 1;
    
    
    plot(robotCurrentPose(1), robotCurrentPose(2), 'bo', 'MarkerSize', 2)
    plot(laPoint(1), laPoint(2), 'r*', 'MarkerSize', 2)
%     pause;
    
    time = time + sampleTime;
    distanceToGoal = norm(pathGoal - robotCurrentPose(1:2));
end
hold off;

timeInterval = 0:0.1:time;

% figure(10); grid on;
% plot(timeInterval, lateralTrackingError(1:i-1), 'b')
figure(11); grid on;
plot(path(:,1), path(:,2), 'k--d', odomHistory(1:i-1,1), odomHistory(1:i-1,2), 'r')
title("LA: 0.2 m")



% plot(odomHistory(1:i-1), timeInterval, 'r')
