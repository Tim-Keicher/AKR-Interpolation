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
[q,~] = trapveltraj(frankaWaypoints,numSamples)

visualisierung = "on";  % on/off
figure
set(gcf,'Visible','on');
rc = rateControl(sampleRate);
trueb = 0
while trueb < 2

    for i = 1:numSamples
        
        show(robot,q(:,i),FastUpdate=true,PreservePlot=false,Visuals=visualisierung);
        waitfor(rc);
        
    end
     
    trueb = trueb +1
end

