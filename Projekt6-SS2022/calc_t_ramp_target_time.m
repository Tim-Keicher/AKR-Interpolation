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

function [ta, tv, te] = calc_t_ramp_target_time(t_target, s_e, v_max, a_max)
    % Gibt error Nachricht zurück, falls eine Strecke negativen ist
    for i = 1:numel(s_e)
        if s_e(i) < 0
            error("Bei der Zeitberechnung wurden negative Strecken übergeben, sie " + ...
                "müssen jedoch immer positive sein.")
        end
    end

    % Berechnung der Zeitintervalle für jedes Bahnsegment
    t_e = t_target*(s_e/sum(s_e));
    t_e(2,:) = t_e(1,:);
    
    % t_a = t_e./4;   % definiert (manuel)

    a = -1 / a_max(1);
    b = t_e(1,:);
    c = -s_e;

    v1 = zeros(1, length(s_e));
    v2 = zeros(1, length(s_e));
    for i = 1:length(s_e)
        D = b(i)^2 - 4 * a * c(i)
        if D < 0
            error("Weg kann nicht gefahren werden, neg. Wurzel! Maximale Beschleunigung zu gering")
        end
    
        v1(i) = (-b(i) + sqrt(D)) / (2 * a);
        v2(i) = (-b(i) - sqrt(D)) / (2 * a);
    end

    te = t_e(1,:);
    ta = v1 ./ a_max(1);
    tv = v2 ./ a_max(1);

    % Berechnung der maximalen Geschwindigkeit abh. der maximalen Beschleunigung
    % v_max_calc = sqrt(s_e.*a_max);
    % v_max_calc_t = s_e.*(t_e - t_a);
    
    % Bestimmung der Maximalgeschwindigkeiten durch Auswahl das kleinsten
    % Wertes aus der Berechnung und Angabe
    % vm = zeros(2,numel(s_e));
    % for i = 1:numel(v_max_calc_a(1,:))
    %     vm(1,i) = min([v_max_calc_a(1,i) v_max_calc_t(1,i) v_max(1)]);
    %     if numel(s_e) > 1
    %         vm(2,i) = min([v_max_calc_a(2,i) v_max_calc_t(2,i) v_max(2)]);
    %     end
    % end
    
    % Berechnung der Zeitenabschnitte
    % t_a = vm ./ a_max;
    % t_v = t_e - t_a;
    % 
    % % Prueft, ob die Bahn mit den Angegebenen Parametern gefahren werden kann
    % for i = 1:numel(s_e)
    %     if t_v(i) < t_a(i)
    %         % Error erscheint und Programm wird abgebrochen
    %         error("Mit den agegebenen Parametern kann der Weg nicht gefahren werden")
    %     end
    % end

    % te = t_e(1,:);
    % tv = t_v(1,:);
    % ta = t_a(1,:);
end