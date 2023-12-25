function designAxes(app)
   
    % Ausrichtung der Axes ihrer Sichtbarkeit nach:
    % die erste Sichtbare Axes wird oben links gesetzt,
    % die zweite oben rechts
    % die dritte unten links
    % die vierte unten rechts
    %
    % Alle nicht vorhandenen Axen werden nicht gesetzt und
    % das Layout wird auf die Anzahl Sichtbarer Axen angepasst
    
    % Neuanordnung der Sichtbaren Axen
    cnt = 1;
    if app.BewegungssimulationCheckBox.Value
        app.robotSimUIAxes.Layout.Row = 1;
        app.robotSimUIAxes.Layout.Column = 1;
        cnt = cnt + 1;
    end
    if app.WinkelVerlaufCheckBox.Value
        app.angleUIAxes.Layout.Row = 1;
        app.angleUIAxes.Layout.Column = cnt;
        cnt = cnt + 1;
    end
    if app.GeschwVerlaufCheckBox.Value
        if mod(cnt,2) == 0
            app.velUIAxes.Layout.Row = cnt / 2;
            app.velUIAxes.Layout.Column = 2;
        else
            app.velUIAxes.Layout.Row = round(cnt/2);
            app.velUIAxes.Layout.Column = 1;
        end
        cnt = cnt + 1;
    end
    if app.BeschlVerlaufCheckBox.Value
        if mod(cnt,2) == 0
            app.accUIAxes.Layout.Row = cnt / 2;
            app.accUIAxes.Layout.Column = 2;
        else
            app.accUIAxes.Layout.Row = round(cnt/2);
            app.accUIAxes.Layout.Column = 1;
        end
        cnt = cnt + 1;
    end
    
    % Grid-Layout werden entsprechend der sichtbaren Axes angepassen
    switch (cnt - 1)
        case 1
            % Eine sichtbare Axes: Ganzes Layout verwenden
            app.GridLayoutLeft.RowHeight = {'1x', '0x'};
            app.GridLayoutLeft.ColumnWidth = {'1x', '0x'};
        case 2
            % Zwei sichtbare Axes: 2x1 Layout
            app.GridLayoutLeft.RowHeight = {'1x', '0x'};
            app.GridLayoutLeft.ColumnWidth = {'1x', '1x'};
        case 3
            % Drei sichtbare Axes: 2x2 Layout
            app.GridLayoutLeft.RowHeight = {'1x', '1x'};
            app.GridLayoutLeft.ColumnWidth = {'1x', '1x'};
        case 4
            % Vier sichtbare Axes: 2x2 Layout
            app.GridLayoutLeft.RowHeight = {'1x', '1x'};
            app.GridLayoutLeft.ColumnWidth = {'1x', '1x'};
        otherwise
            warning("Visible Axes: case not available!");
    end

    % Setzen der Sichtbarkeit für alle Axes
    app.robotSimUIAxes.Visible = app.BewegungssimulationCheckBox.Value;
    app.angleUIAxes.Visible = app.WinkelVerlaufCheckBox.Value;
    app.velUIAxes.Visible = app.GeschwVerlaufCheckBox.Value;
    app.accUIAxes.Visible = app.BeschlVerlaufCheckBox.Value;
end