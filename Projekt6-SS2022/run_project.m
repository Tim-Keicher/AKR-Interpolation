function run_project(app, ax1, ax2, ax3, ax4, konfig, vm, am ,t_target, gif_erstellen_flag)  
    % Flag um angehaltene Interrupts zu beenden wird zurueckgesetzt
    app.canle_old_runs_flag = 0;

    syms t  real % reale Variable t definiert (ohne Wert)
    
    % Anpassbare Parameter
    % konfig = 0;        % Bestimmt die Konfiguration des Roboters (0: Standard, 1: Umgekehrte Konfiguration)
    %t_target = 0;       % ! Um die Fahrt nach Zeitvorgabe zu deaktivieren muss t_target = 0 gesetzt werden
    %vm = [3, 3]';       % Festlegung der maximale Gelenkgeschwindigkeiten für J1 und J2
    %am = [0.2, 0.2]';   % Festlegung der maximalen Gelenkbeschleunigungen für J1 und J2
    
    % Beispiel mit t_target
    % t_target = 160;
    % vm = [3, 3]';
    % am = [0.5, 0.5]';
    
    l1 = 40; % Armteillänge Arm 1
    l2 = 40; % Armteillänge Arm 2
    
    t_start = 0; % Start zum Zeitpunkt 0
    
    % Koordinaten der Anzufahrenden Punkte
    p_punkte = [-40 40;
                -40 65;
                 40 65;
                 40 15;
                -40 15;
                -40 40]';

    % p_punkte = [-40 40;
    %             -40 52.5;
    %             -20 65;
    %              20 65;
    %              40 52.5;
    %              40 27.5;
    %              20 15;
    %             -20 15;
    %             -40 27.5
    %             -40 40]';

    % ---------------------------------------------------------------
    % Start der Berechnungen

    if t_target == 0    % Fahrt nach vorgegebenen Geschwindigkeiten & Beschleunigungen
        % Initialisierung der Variablen
        s_ep = zeros(length(p_punkte)-1, 2);
        ta = zeros(length(p_punkte)-1, 2);
        tv = zeros(length(p_punkte)-1, 2);
        te = zeros(length(p_punkte)-1, 2);
        for j = 1:length(s_ep)
            % Berechnung der Strecke der Bahnsegmente
            if (p_punkte(2,j+1) - p_punkte(2, j)) == 0 % Unterscheidung zwischen Achsbewegungsrichtung
                s_ep(j,:) = [sqrt((p_punkte(1, j+1) - p_punkte(1, j))^2 + (p_punkte(2, j+1) - p_punkte(2, j))^2);
                            0]; % x-Richtung
            else
                s_ep(j,:) = [0;
                            sqrt((p_punkte(1, j+1) - p_punkte(1, j))^2 + (p_punkte(2, j+1) - p_punkte(2, j))^2)]; % y-Richtung
            end
            % Berechnung der Verfahrenszeiten je Bahnsegment
            [ta_temp, tv_temp, te_temp] = calc_t_ramp(s_ep(j,:), vm, am); % Funktion berechnet aus vm, bm und se die Zeiten tb, tv und te
            ta(j,:) = ta_temp;
            tv(j,:) = tv_temp;
            te(j,:) = te_temp;
        end
    else    % Fahrt nach Zeitvorgabe
        % Berechnung der Bahnsegmentaengen
        s_e = zeros(1,numel(p_punkte(1,:))-1);
        for i = 2:numel(p_punkte(1,:))
            s_e(i-1) = sqrt((p_punkte(1,i)-p_punkte(1,i-1))^2 + (p_punkte(2,i)-p_punkte(2,i-1))^2);
        end
    
        [ta_temp, tv_temp, te_temp, err] = calc_t_ramp_target_time(t_target, s_e, vm, am);
    
        % Auf Fehler pruefen
        if ~isempty(err)
            app.SystemStateLamp.Color = [0.90,0.90,0.90];   % Systemlampe zuruecksetzten
            
            % Setzten der entsprechnenden Signallampe
            app.ParamLamp.Color = [1, 0, 0];
            drawnow()

            warning(err)    % Warning für den Fehler in der Konsole ausgeben

            return;
        else    % Zuruecksetzten der Signallampen der Fehler, falls Fehlerstatus von vorheriger Runde vorhanden. 
            app.ParamLamp.Color = [0.90,0.90,0.90];
            disp('no error')
        end

        % Initialisierung der Variablen
        ta = zeros(length(p_punkte)-1, 2);
        tv = zeros(length(p_punkte)-1, 2);
        te = zeros(length(p_punkte)-1, 2);
        for j = 1:length(s_e)
            % Berechnung der Verfahrenszeiten je Bahnsegment
            if (p_punkte(2,j+1) - p_punkte(2, j)) == 0 % Unterscheidung zwischen Achsbewegungsrichtung
                ta(j,:) = [ta_temp(j); 0];
                tv(j,:) = [tv_temp(j); 0];
                te(j,:) = [te_temp(j); 0];
            else
                ta(j,:) = [0; ta_temp(j)];
                tv(j,:) = [0; tv_temp(j)];
                te(j,:) = [0; te_temp(j)];
            end
        end
    end
    
    [p_1, v_1, a_1, glg_a_1, glg_v_1, glg_s_1, glg_a_p1, glg_v_p1, glg_s_p1, zeit_1] = calc_p_linear(p_punkte(:,1), p_punkte(:,2), am, t_start, ta(1,:), tv(1,:), te(1,:));
    [p_2, v_2, a_2, glg_a_2, glg_v_2, glg_s_2, glg_a_p2, glg_v_p2, glg_s_p2, zeit_2] = calc_p_linear(p_punkte(:,2), p_punkte(:,3), am, max(te(1,:)), ta(2,:), tv(2,:), te(2,:));
    [p_3, v_3, a_3, glg_a_3, glg_v_3, glg_s_3, glg_a_p3, glg_v_p3, glg_s_p3, zeit_3] = calc_p_linear(p_punkte(:,3), p_punkte(:,4), am, max(te(1,:))+max(te(2,:)), ta(3,:), tv(3,:), te(3,:));
    [p_4, v_4, a_4, glg_a_4, glg_v_4, glg_s_4, glg_a_p4, glg_v_p4, glg_s_p4, zeit_4] = calc_p_linear(p_punkte(:,4), p_punkte(:,5), am, max(te(1,:))+max(te(2,:))+max(te(3,:)), ta(4,:), tv(4,:), te(4,:));
    [p_5, v_5, a_5, glg_a_5, glg_v_5, glg_s_5, glg_a_p5, glg_v_p5, glg_s_p5, zeit_5] = calc_p_linear(p_punkte(:,5), p_punkte(:,1), am, max(te(1,:))+max(te(2,:))+max(te(3,:))+max(te(4,:)), ta(5,:), tv(5,:), te(5,:));
    
    zeit = vertcat(zeit_1, zeit_2, zeit_3, zeit_4, zeit_5);
    v_gesamt = vertcat(v_1, v_2, v_3, v_4, v_5);
    a_gesamt = vertcat(a_1, a_2, a_3, a_4, a_5);
    p_gesamt = vertcat(p_1, p_2, p_3, p_4, p_5);                                                    % Zusammenführen von alle Streckenpunkten untereinander in einer Matrix
    te_gesamt = max(te(1,:)) + max(te(2,:)) + max(te(3,:)) + max(te(4,:)) + max(te(5,:));           % Berechnung Gesamtverfahrzeit 

    [p_J1, p_J2, winkel] = calc_axis(l1, l2, p_gesamt, konfig); % Bestimmung q1, q2 und daraus Position J2 und TCP (Überprüfung)

    % --------------------------------------------------------------------
    % plots

    % Name fuer das Gif wird immer erstellt. Dadurch kann ein Gif auch erst während der Fahrt erstellt werden
    filename = sprintf('Animations/%s-animation.gif', datestr(now,'yyyy-mm-dd-HHMMSS'));

    % Erstellung der Roboter Animation + Rechteck
    cla(ax1)    % Plot loeschen, falls noch aus vorherigen durchläufen vorhanden
    if app.BewegungssimulationCheckBox.Value
        hold(ax1, 'on');
        plot(ax1, p_punkte(1, :), p_punkte(2, :), 'k-'); % Rechteck zeichnen
        rr_robot = animatedline(ax1, 'Marker', 'o','Color', 'b'); % Roboter Plot mit ersten Werten erstellen
    
        q1 = quiver(ax1, p_gesamt(1, 1), p_gesamt(1, 2), v_gesamt(1, 1), v_gesamt(1, 2), 'linewidth', 2, 'color', 'g');	% Geschwindigkeitsvektor erstellen
        set(q1, 'AutoScale', 'on', 'AutoScaleFactor', 5, 'MaxHeadSize', 5);
        q2 = quiver(ax1, p_gesamt(1, 1), p_gesamt(1, 2), a_gesamt(1, 1), a_gesamt(1, 2), 'linewidth', 2, 'color', 'r');  % Bescghleunigungsvektor erstellen
        set(q2, 'AutoScale', 'on', 'AutoScaleFactor', 25, 'MaxHeadSize', 5);
        
        if konfig == 0
            legend(ax1, 'RR-Roboter', 'Verfahrweg Rahmenprofil TCP', 'Geschwindigkeitsvektor', 'Beschleunigungsvektor', 'Location', 'southwest')
        else
            legend(ax1, 'RR-Roboter', 'Verfahrweg Rahmenprofil TCP', 'Geschwindigkeitsvektor', 'Beschleunigungsvektor', 'Location', 'southeast')
        end
        axis(ax1, [-100 100 -30 70]);
        hold(ax1, 'off');
    end


    % Erstellung des Winkelverlaufs
    cla(ax2)    % Plot loeschen, falls noch aus vorherigen durchläufen vorhanden
    if app.WinkelVerlaufCheckBox.Value
        hold(ax2, 'on');
        winkel_deg = winkel .* (180)/pi;
        plot(ax2, zeit(:), winkel_deg(:, 1), 'r-'); % Winkelverlauf Gelenk J1 Zeichnen
        plot(ax2, zeit(:), winkel_deg(:, 2), 'b-'); % Winkelverlauf Gelenk J2 Zeichnen
    
        angle_J1 = animatedline(ax2, 'Marker', 'o','Color', 'r'); % Zeichnen der aktuellen Position von Gelenk J1
        angle_J2 = animatedline(ax2, 'Marker', 'o','Color', 'b'); % Zeichnen der aktuellen Position von Gelenk J2
        
        legend(ax2, 'Gelenk J1', 'Gelenk J2', 'Location', 'southwest')
        % axis(ax2, [0 max(te_gesamt) floor(min(winkel_deg(:))/10)*10-20 ceil(max(winkel_deg(:))/10)*10]+20);  % something went wrong
        hold(ax2, 'off');
    end


    % Erstellung des Geschwindigkeitsverlaufs
    cla(ax3)    % Plot loeschen, falls noch aus vorherigen durchläufen vorhanden
    if app.GeschwVerlaufCheckBox.Value
        hold(ax3, 'on');
        xv1 = double(subs(glg_v_p1(2, 1), t, zeit));
        fplot(ax3, glg_v_p1(2, 1), [0, max(te_gesamt)], 'g-'); % Strecke Pstart-P1
        p_v1 = animatedline(ax3, 'Marker', 'o','Color', 'g');  % Aktuelle Geschwindigkeit
        
        xv2 = double(subs(glg_v_p2(1, 1), t, zeit));
        fplot(ax3, glg_v_p2(1, 1), [0, max(te_gesamt)], 'r-'); % Strecke P1-P2
        p_v2 = animatedline(ax3, 'Marker', 'o','Color', 'r');  % Aktuelle Geschwindigkeit
        
        xv3 = double(subs(-glg_v_p3(2, 1), t, zeit));
        fplot(ax3, -glg_v_p3(2, 1), [0, max(te_gesamt)], 'b-'); % Strecke P2-P3
        p_v3 = animatedline(ax3, 'Marker', 'o','Color', 'b');   % Aktuelle Geschwindigkeit
        
        xv4 = double(subs(-glg_v_p4(1, 1), t, zeit));
        fplot(ax3, -glg_v_p4(1, 1), [0, max(te_gesamt)], 'k-'); % Strecke P3-P4
        p_v4 = animatedline(ax3, 'Marker', 'o','Color', 'k');   % Aktuelle Geschwindigkeit
        
        xv5 = double(subs(glg_v_p5(2, 1), t, zeit));
        fplot(ax3, glg_v_p5(2, 1), [0, max(te_gesamt)], 'c-'); % Strecke P4-P5
        p_v5 = animatedline(ax3, 'Marker', 'o','Color', 'c');  % Aktuelle Geschwindigkeit
    
        legend(ax3, 'Strecke Pstart-P1', '', 'Strecke P1-P2', '', 'Strecke P2-P3', '', 'Strecke P3-P4', '', 'Strecke P4-PStart', '', 'Location', 'southwest')
        axis(ax3, [0 max(te_gesamt) -4 4]);
        hold(ax3, 'off');
    end


    % Erstellung des Beschleunigungsverlaufs
    cla(ax4)    % Plot loeschen, falls noch aus vorherigen durchläufen vorhanden
    if app.BeschlVerlaufCheckBox.Value
        hold(ax4, 'on');
        xa1 = double(subs(glg_a_p1(2, 1), t, zeit));
        fplot(ax4, glg_a_p1(2, 1), [0, max(te_gesamt)], 'g-'); % Strecke Pstart-P1
        p_a1 = animatedline(ax4, 'Marker', 'o','Color', 'g');  % Aktuelle Beschleunigung
        
        xa2 = double(subs(glg_a_p2(1, 1), t, zeit));
        fplot(ax4, glg_a_p2(1, 1), [0, max(te_gesamt)], 'r-'); % Strecke P1-P2
        p_a2 = animatedline(ax4, 'Marker', 'o','Color', 'r');  % Aktuelle Beschleunigung
        
        xa3 = double(subs(-glg_a_p3(2, 1), t, zeit));
        fplot(ax4, -glg_a_p3(2, 1), [0, max(te_gesamt)], 'b-'); % Strecke P2-P3
        p_a3 = animatedline(ax4, 'Marker', 'o','Color', 'b');   % Aktuelle Beschleunigung
        
        xa4 = double(subs(-glg_a_p4(1, 1), t, zeit));
        fplot(ax4, -glg_a_p4(1, 1), [0, max(te_gesamt)], 'k-'); % Strecke P3-P4
        p_a4 = animatedline(ax4, 'Marker', 'o','Color', 'k');   % Aktuelle Beschleunigung
        
        xa5 = double(subs(glg_a_p5(2, 1), t, zeit));
        fplot(ax4, glg_a_p5(2, 1), [0, max(te_gesamt)], 'c-'); % Strecke P4-P5
        p_a5 = animatedline(ax4, 'Marker', 'o','Color', 'c');  % Aktuelle Beschleunigung
    
        legend(ax4, 'Strecke Pstart-P1', '', 'Strecke P1-P2', '', 'Strecke P2-P3', '', 'Strecke P3-P4', '', 'Strecke P4-PStart', '', 'Location', 'southwest')
        axis(ax4, [0 max(te_gesamt) -1.5 1.5]);
        hold(ax4, 'off');
    end


    % Aktualisierung der Dartsellungen
    steps = size(p_gesamt, 1);
    for i = 1:steps

        if app.BewegungssimulationCheckBox.Value
            % Aktualiesierung der Roboterarme
            clearpoints(rr_robot);  % löscht die Zeichnung der SChleife 1-1
            addpoints(rr_robot, [0, p_J1(i, 1)], [0, p_J1(i, 2)]);                      % Armteil 1
            addpoints(rr_robot, [p_J1(i, 1), p_J2(i, 1)], [p_J1(i, 2), p_J2(i, 2)]);    % Armteil 2
            
            % Aktualisierung des Geschwindigkeits- und Beschleunigungsvektor
            set(q1, 'xdata', p_gesamt(i, 1), 'ydata', p_gesamt(i, 2), 'udata', v_gesamt(i, 1), 'vdata', v_gesamt(i, 2))
            set(q2, 'xdata', p_gesamt(i, 1), 'ydata', p_gesamt(i, 2), 'udata', a_gesamt(i, 1), 'vdata', a_gesamt(i, 2))
        end

        if app.WinkelVerlaufCheckBox.Value
            % Aktualisierung der aktuellen Gelenkposition J1
            clearpoints(angle_J1);  % löscht die Zeichnung der SChleife 1-1
            addpoints(angle_J1, zeit(i), winkel_deg(i, 1));
            
            % Aktualisierung der aktuellen Gelenkposition J2
            clearpoints(angle_J2);  % löscht die Zeichnung der SChleife 1-1
            addpoints(angle_J2, zeit(i), winkel_deg(i, 2));
        end

        % Aktualisierung des Geschwindigkeitsverlaufs
        if app.GeschwVerlaufCheckBox.Value
            clearpoints(p_v1);
            addpoints(p_v1, zeit(i), xv1(i));
    
            clearpoints(p_v2);
            addpoints(p_v2, zeit(i), xv2(i));
    
            clearpoints(p_v3);
            addpoints(p_v3, zeit(i), xv3(i));
    
            clearpoints(p_v4);
            addpoints(p_v4, zeit(i), xv4(i));
    
            clearpoints(p_v5);
            addpoints(p_v5, zeit(i), xv5(i));
        end

        % Aktualisierung des Beschleunigungsverlaufs
        if app.BeschlVerlaufCheckBox.Value
            clearpoints(p_a1);
            addpoints(p_a1, zeit(i), xa1(i));
    
            clearpoints(p_a2);
            addpoints(p_a2, zeit(i), xa2(i));
    
            clearpoints(p_a3);
            addpoints(p_a3, zeit(i), xa3(i));
    
            clearpoints(p_a4);
            addpoints(p_a4, zeit(i), xa4(i));
    
            clearpoints(p_a5);
            addpoints(p_a5, zeit(i), xa5(i));
        end

        drawnow();

        % Gif erstelln, wenn falg gesetzt ist
        if gif_erstellen_flag.Value == 1
            frame = getframe(app.UIFigure);
            im = frame2im(frame);
            [imind, cm] = rgb2ind(im, 256);
            if i == 1
                imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
            else
                imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
            end
            % pause(0.01);
        end

                % ...
        while app.pause_flag == 1
            pause(0.01)
            if app.canle_old_runs_flag == 1
                return;
            end
        end
    end

    % Flag um angehaltene Interrupts zu beenden wird gesetzt 
    app.canle_old_runs_flag = 1;
end