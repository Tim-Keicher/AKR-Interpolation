beep off
close all
%robot = loadrobot( 'atlas')
%show(robot);
% robot.DataFormat = "row";
% config = homeConfiguration(robot)
% config(2).JointPosition = pi/2;
% show(robot,config);

% robot1 = loadrobot("omronEcobra600");
% showdetails(robot1)
% config = homeConfiguration(robot1)
% show(robot1);
%%
% config(1).JointPosition   Rotation um eigene achse            Gelenk J1
% config(2).JointPosition   Rotation des gelnks                 Gelenk J1
% config(3).JointPosition   Rotation des gelnks                 Gelenk J2
% config(4).JointPosition   Rotation des Manipulator Gelenks    Gelenk J3
% show(robot,randomConfiguration(robot)); % Erzeugt zufällige Roboter Position
%%
% winkel1 = winkel(:,1);
% winkel2= winkel(:,2);
% if size(winkel1,1) ~= size(winkel2,1)
%     error("ERROR!!!!!!")
% end
% tpts = 0:4;
% sampleRate = size(winkel1,1) ;
% tvec = tpts(1):1/sampleRate:tpts(end);
% numSamples = length(tvec);
% 
% robot = loadrobot('omronEcobra600',DataFormat='column'); % quanserQArm 
% 
% gen_waypoints = zeros(4, size(winkel1, 1)-1);
% 
% for i = 1:size(winkel1, 1)
%     gen_waypoints(:, i) = [0; winkel1(i, 1) - pi/2; winkel2(i, 1)-pi/2 ;  0];
% end
% %rng default
% 
% % %% Original
% % robot.homeConfiguration robot.homeConfiguration
% % frankaWaypoints = gen_waypoints
% % frankaTimepoints = linspace(tvec(1),tvec(end),4); 
% % [q,qd] = trapveltraj(frankaWaypoints,numSamples);
% % %%
% 
% frankaWaypoints = gen_waypoints
% %frankaWaypoints=[[0, -1.53248023105472, -0.0390500675991217, 0]', [0, -0.888766093087712, 1.14699686008123, 0]']
% %frankaTimepoints = linspace(tvec(1),tvec(end),160); 
% [q,qd] = trapveltraj(frankaWaypoints,numSamples);
% 
% figure
% set(gcf,'Visible','on');
% rc = rateControl(sampleRate);
% trueb = 0
% while trueb < 0
% 
%     for i = 1:numSamples
%         tNow= numSamples(i);
%         show(robot,q(:,i),FastUpdate=true,PreservePlot=false);
%         waitfor(rc);
%         configNow = interp1(tTask,stateTask(:,1:numJoints),tNow);
%         poseNow = getTransform(robot,configNow,endEffector);
%         taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
%         drawnow;
%     end
%     trueb = trueb +1
% end

%robot = loadrobot("omronEcobra600");
% showdetails(robot)
% config = homeConfiguration(robot)
% show(robot);
%
winkelJ1 = winkel(:,1);
winkelJ2 = winkel(:,2);

if size(winkelJ1,1) ~= size(winkelJ2,1)
    disp("ERROR!")
    return
end
tpts = 0:4;
sampleRate = size(winkelJ1,1) ;
tvec = tpts(1):1/sampleRate:tpts(end);
numSamples = length(tvec);

robot = loadrobot('omronEcobra600',DataFormat='column'); %frankaEmikaPanda,    quanserQArm

gen_waypoints = zeros(4, size(winkelJ1, 1)-1);

for i = 1:size(winkelJ1, 1)
    gen_waypoints(:, i) = [winkelJ1(i, 1);winkelJ2(i, 1) ; 0;  0];

end
frankaWaypoints = gen_waypoints;
[q,~] = trapveltraj(frankaWaypoints,numSamples);


figure
set(gcf,'Visible','on');
rc = rateControl(sampleRate);
trueb = 0
while trueb < 5

    for i = 1:numSamples

        show(robot,q(:,i),FastUpdate=true,PreservePlot=false);
        waitfor(rc);
    end
    trueb = trueb +1
end
