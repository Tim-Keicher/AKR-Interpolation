function [] = visualisation_scara(winkel, record)

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

robot = loadrobot('omronEcobra600',DataFormat='column'); %quanserQArm

gen_waypoints = zeros(4, size(winkelJ1, 1)-1);

for i = 1:size(winkelJ1, 1)
    gen_waypoints(:, i) = [winkelJ1(i, 1);winkelJ2(i, 1) ; 0;  0];

end
frankaWaypoints = gen_waypoints;
[q,~] = trapveltraj(frankaWaypoints,numSamples);

 

%plot3(p_J2(:,1), p_J2(:,2),p_J2(:,3), 'o', 'Color', 'b'); % Initialisierung des Plots
%hold on;
figure
set(gcf,'Visible','on');
rc = rateControl(sampleRate);

if record == true
    filename = sprintf('Animations/%s-animation.gif', datestr(now,'yyyy-mm-dd-HHMMSS'));
    f = figure('Name', 'Animation','Visible','on','Position',[0 0 1440 810]); % Figur erstellen
    for i = 1:numSamples
        show(robot,q(:,i),FastUpdate=true, PreservePlot = false,Frames="off",Collisions="off");
        waitfor(rc);

        frame = getframe(f);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        if i == 1
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
        end
    %pause(0.01);
    end
else
    for i = 1:numSamples
        show(robot,q(:,i),FastUpdate=true, PreservePlot = false,Frames="off",Collisions="off");
        waitfor(rc);
    end
end

end


    

