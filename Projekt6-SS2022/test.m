function [x, y] = test(steps)
    x = 0:steps:(360/180)*pi;
    y = sin(x);

    p_punkte = [-40 40;
            -40 65;
             40 65;
             40 15;
            -40 15;
            -40 40]';
end