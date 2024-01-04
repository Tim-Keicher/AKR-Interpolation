robot = loadrobot('omronEcobra600','DataFormat','row','Gravity',[0 0 -9.81]);
currentRobotJConfig = homeConfiguration(robot);
numJoints = numel(currentRobotJConfig);
endEffector = "EndEffector_Link"

for i=1:length(trajTimes)
    % Current time 
    tNow= trajTimes(i);
    % Interpolate simulated joint positions to get configuration at current time
    configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);
    poseNow = getTransform(robot,configNow,endEffector);
    show(robot,configNow,'PreservePlot',false,'Frames','off');
    taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
    drawnow;
end