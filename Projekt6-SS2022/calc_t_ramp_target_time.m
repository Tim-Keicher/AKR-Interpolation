% CALC_T_RAMP_TARGET_TIME - Berechnung der Zeitintervalle für ein Rampenprofil
%                           mit Angabe einer vorgegebenen Gesamtzeit
%
%   [ta, tv, te] = calc_t_ramp_target_time(t_target, s_e, v_max, a_max)
%
% Eingabe:
%   t_target - Zielverfahrenszeit
%   s_e - Vektor mit den Längen der Bahnsegmente
%   v_max - Maximale Geschwindigkeiten für jede Achse
%   a_max - Maximale Beschleunigungen für jede Achse
%
% Ausgabe:
%   ta - Beschleunigungszeit für jedes Bahnsegment
%   tv - Konstante Geschwindigkeitszeit für jedes Bahnsegment
%   te - Gesamtzeit für jedes Bahnsegment (Beschleunigung, konstante Geschwindigkeit, 
%        Bremsung)
%
% Beispiel:
%   t_target = 10;
%   s_e = [5 10 5];
%   v_max = [2 2];
%   a_max = [1 1];
%   [ta, tv, te] = calc_t_ramp_target_time(t_target, s_e, v_max, a_max);

function [ta, tv, te, err] = calc_t_ramp_target_time(t_target, s_e, v_max, a_max)
    err = '';

    % Gibt error Nachricht zurück, falls eine Strecke negativen ist
    for i = 1:numel(s_e)
        if s_e(i) < 0
            err = "Bei der Zeitberechnung wurden negative Strecken übergeben, sie " + ...
                "müssen jedoch immer positive sein.";
        end
    end

    % Berechnung der Zeitintervalle für jedes Bahnsegment
    t_e = t_target * (s_e / sum(s_e));
    t_e(2,:) = t_e(1,:);
    
    % Parameter der Mitternachtsformel bestimmen
    a = -1 / a_max(1);
    b = t_e(1,:);
    c = -s_e;

    % Berechnung der Geschwindigkeiten in Bezug auf die Beschleunigungen
    v1 = zeros(1, length(s_e));
    v2 = zeros(1, length(s_e));
    for i = 1:length(s_e)
        D = b(i)^2 - 4 * a * c(i);
        if D < 0
            err = "Weg kann nicht gefahren werden, neg. Wurzel!";
        end
    
        % Loesen der quadratischen Gleichung
        v1(i) = (-b(i) + sqrt(D)) / (2 * a);
        v2(i) = (-b(i) - sqrt(D)) / (2 * a);

        if v1(i) > v_max(1)
            err = "Die maximale Geschwindigkeit ist zu gering!";
        end
    end

    % Rückgabewerte
    te = t_e(1,:);
    ta = v1 ./ a_max(1);
    tv = v2 ./ a_max(1);
end