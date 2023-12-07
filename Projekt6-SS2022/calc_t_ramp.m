% CALC_T_RAMP - Berechnung der Zeitintervalle für ein Rampenprofil unter
%               angabe der Geschwindigkeit und Beschleunigung
%
%   [ta, tv, te] = calc_t_ramp(se, vm, am)
%
% Eingabe:
%   se - Vektor mit den Längen der Bahnsegmente
%   vm - Geschwindigkeiten für jede Achse
%   am - Beschleunigungen für jede Achse
%
% Ausgabe:
%   ta - Vektor mit Beschleunigungsendzeitt für x und y Richtung [x, y]
%   tv - Vektor mit Geschwindigkeitsendzeit für x und y Richtung [x, y]
%   te - Vektor mit Gesamtzeit für x und y Richtung [x, y]
%
% Beispiel:
%   se = [10, 5];
%   vm = [2, 1];
%   am = [1, 0.5];
%   [ta, tv, te] = calc_t_ramp(se, vm, am);

function [ta, tv, te] = calc_t_ramp(se, vm, am)
    syms t
    for i = 1:size(vm, 1)
        if se(i) > 0 % Abfangen von negativen Eingaben bei Wegstrecke
            if vm(i) > (se(i) * am(i))^0.5 % Prüfung ob vm höher ist als vm,max
                vm(i) = (se(i) * am(i))^0.5; % vm auf vm,max setzen
                disp('Programmierte Geschwindigkeit wird nicht erreicht')
            end
            ta(i) = vm(i) / am(i);
            te(i) = se(i) / vm(i) + ta(i);
            tv(i) = te(i) - ta(i);
        else
            ta(i) = 0; te(i) = 0; tv(i) = 0; % Nullsetzen der Ausgabewerte im Fehlerfall
        end
    end
end
