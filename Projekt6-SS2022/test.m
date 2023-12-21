function test(app)
    clc;
    
    p_punkte = [-40 40;
                -40 65;
                 40 65;
                 40 15;
                -40 15;
                -40 40]';
    
    axis(app, [-100 100 -30 70]);
    plot(app, p_punkte(1, :), p_punkte(2, :), 'k-'); % Rechteck zeichnen
    hold(app, 'on');
    
    x = -40:0.5:40;
    y = ones(1, length(x)) * 65;
    
    % plot(app.robotDrive,x,y);
    
    h = animatedline(app, 'Marker', 'o', 'Color', 'r');
    
    for k = 1:length(x)
        clearpoints(h);
        addpoints(h, x(k), y(k));
        drawnow();
    end
    
    hold(app, 'off');
end