% CALC_AXIS - Berechnung der Gelenkwinkel für einen 2-Gelenk-Roboterarm
%
%   [p_J2, p_J_TCP, winkel] = calc_axis(l1, l2, p_TCP)
%
% Eingabe:
%   l1 - Länge des ersten Gelenkarms
%   l2 - Länge des zweiten Gelenkarms
%   p_TCP - Position des TCP (Werkzeugzentrum) im kartesischen Raum
%
% Ausgabe:
%   p_J2 - Position des zweiten Gelenks im kartesischen Raum
%   p_J_TCP - Position des TCP im kartesischen Raum
%   winkel - Gelenkwinkel (Alpha, Beta) in Radiant für jedes Punktpaar
%
% Beispiel:
%   l1 = 2;
%   l2 = 1;
%   p_TCP = [2, 2, 0];
%   [p_J2, p_J_TCP, winkel] = calc_axis(l1, l2, p_TCP);

function [p_J2, p_J_TCP, winkel] = calc_axis(l1, l2, p_TCP)
    p_J2 = zeros(size(p_TCP, 1), 3);    % 0er Array in Größe P_TCP,3
    p_J_TCP = zeros(size(p_TCP, 1), 3); % 0er Array in Größe P_TCP,3
    winkel = zeros(size(p_TCP, 1), 2);  % 0er Array in Größe P_TCP,2

    NumPoints = size(p_TCP, 1);

    for i = 1:NumPoints
        % Alpha berechnen
        alpha = atan2(p_TCP(i, 2), p_TCP(i, 1));
        p = sqrt(p_TCP(i, 1)^2 + p_TCP(i, 2)^2);

        % Überprüfung, ob Roboter Punkt erreichen kann
        if (p > l1 + l2) || (p < l1 - l2)
            fprintf('Kann Position nicht erreichen')
            return
        end
        
        % Beta und chi berechnen
        beta = acos(((l1^2 + p^2 - l2^2) / (2 * l1 * p)));
        chi = acos(((l1^2 + l2^2 - p^2) / (2 * l1 * l2)));
        winkel(i, 1) = (alpha - beta);
        winkel(i, 2) = (pi - chi);

        % Rotationsmatrix Aufstellen von K1 zu K0
        R_K1_2_K0 = [cos(winkel(i, 1)), -sin(winkel(i, 1)), 0;
                     sin(winkel(i, 1)), cos(winkel(i, 1)), 0;
                     0, 0, 1];
        p11_K0 = R_K1_2_K0 * [l1; 0; 0]; % Position J2 

        % Rotationsmatrix Aufstellen von K2 zu K1
        R_K2_2_K1 = [cos(winkel(i, 2)), -sin(winkel(i, 2)), 0;
                     sin(winkel(i, 2)), cos(winkel(i, 2)), 0;
                     0, 0, 1];

        % Position TCP (Überprüfung)
        p21_K0 = R_K1_2_K0 * R_K2_2_K1 * [l2; 0; 0] + p11_K0;
        p_J2(i, :) = p11_K0';
        p_J_TCP(i, :) = p21_K0';
    end
end
