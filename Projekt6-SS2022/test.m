function test(app, konfig)
    clc;
    
    syms t  real % reale Variable t definiert (ohne Wert)
    
    % Anpassbare Parameter
    % konfig = 0;         % Bestimmt die Konfiguration des Roboters (0: Standard, 1: Umgekehrte Konfiguration)
    t_target = 0;       % ! Um die Fahrt nach Zeitvorgabe zu deaktivieren muss t_target = 0 gesetzt werden
    vm = [3, 3]';       % Festlegung der maximale Gelenkgeschwindigkeiten für J1 und J2
    am = [0.2, 0.2]';   % Festlegung der maximalen Gelenkbeschleunigungen für J1 und J2
    
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
    
        [ta_temp, tv_temp, te_temp] = calc_t_ramp_target_time(t_target, s_e, vm, am);
    
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
    % plot

    axis(app, [-100 100 -30 70]);
    plot(app, p_punkte(1, :), p_punkte(2, :), 'k-'); % Rechteck zeichnen
    hold(app, 'on');
    grid(app, 'on')

    rr_robot = animatedline(app, 'Marker', 'o','Color', 'b'); % Roboter Plot mit ersten Werten erstellen
    % q1 = animatedline(app);
    q1 = quiver(app, p_gesamt(1, 1), p_gesamt(1, 2), v_gesamt(1, 1), v_gesamt(1, 2), 'linewidth', 2, 'color', 'g');	% Geschwindigkeitsvektor erstellen
    set(q1, 'AutoScale', 'on', 'AutoScaleFactor', 5, 'MaxHeadSize', 5);
    q2 = quiver(app, p_gesamt(1, 1), p_gesamt(1, 2), a_gesamt(1, 1), a_gesamt(1, 2), 'linewidth', 2, 'color', 'r');  % Bescghleunigungsvektor erstellen
    set(q2, 'AutoScale', 'on', 'AutoScaleFactor', 25, 'MaxHeadSize', 5);

    tic
    steps = size(p_gesamt, 1);
    for i = 1:steps
        clearpoints(rr_robot);  % löscht die Zeichnung der SChleife 1-1
        addpoints(rr_robot, [0, p_J1(i, 1)], [0, p_J1(i, 2)]);                      % Armteil 1
        addpoints(rr_robot, [p_J1(i, 1), p_J2(i, 1)], [p_J1(i, 2), p_J2(i, 2)]);    % Armteil 2
        
        set(q1, 'xdata', p_gesamt(i, 1), 'ydata', p_gesamt(i, 2), 'udata', v_gesamt(i, 1), 'vdata', v_gesamt(i, 2))
        set(q2, 'xdata', p_gesamt(i, 1), 'ydata', p_gesamt(i, 2), 'udata', a_gesamt(i, 1), 'vdata', a_gesamt(i, 2))

        drawnow();
    end
    elapsed_time = toc;

    % Display the elapsed time
    disp(['Elapsed time: ' num2str(elapsed_time) ' seconds'])
    
    hold(app, 'off');
    disp('finished')
end