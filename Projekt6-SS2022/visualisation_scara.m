function [] = visualisation_scara(winkel, record)
% In der Funktion wird der Scara Roboter als 3D modell visualiesiert.
% hierbei kann der Anwender entscheiden ob er das 3D modell aufnahemen
% möchte und als GIF speichern über die Boolen Flag "record".
%
% Input:
% winkel - ist ein zwei dim. Array mit den winkeln für das Rotorgelenk J1
%          und J2 
% record - boolen Variabel die bestimmt ob das 3D model als gif
%          aufgezeichnet werden soll oder nicht 

% winkel J1 und J2 zuweisen anhand der Spaleten aus dem 2D array "winkel" 
winkelJ1 = winkel(:,1);
winkelJ2 = winkel(:,2);

if size(winkelJ1,1) ~= size(winkelJ2,1)
    disp("ERROR!")
    return
end

tpts = 0:4;
% Bestimmen der einträge in winkel und diese als anzahl abtastwerte festlegen
sampleRate = size(winkelJ1,1) ;
tvec = tpts(1):1/sampleRate:tpts(end);
numSamples = length(tvec);

% Reinladen des Scara Roboters
robot = loadrobot('omronEcobra600',DataFormat='column');

% Array "gen_waypoints" inizialiesiern mit nullen
gen_waypoints = zeros(4, size(winkelJ1, 1)-1);

for i = 1:size(winkelJ1, 1)
    gen_waypoints(:, i) = [winkelJ1(i, 1);winkelJ2(i, 1) ; 0;  0];
end

frankaWaypoints = gen_waypoints;
[q,~] = trapveltraj(frankaWaypoints,numSamples);

figure
set(gcf,'Visible','on');
rc = rateControl(sampleRate);

if record == true
    % Abspeichern des 3D models in einem GIF
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
        pause(0.01);
    end
else
    % Anzeigen des Roboters anhand der zwei Winkel-Arrays
    for i = 1:numSamples
        show(robot,q(:,i),FastUpdate=true, PreservePlot = false,Frames="off",Collisions="off");
        waitfor(rc);
    end
end

end


    

