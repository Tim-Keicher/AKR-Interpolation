function visibleAxes = getVisibleAxes(app)
    clc;

    visibleAxes = [
        app.BewegungssimulationCheckBox.Value
        app.WinkelVerlaufCheckBox.Value
        app.GeschwVerlaufCheckBox.Value
        app.BeschlVerlaufCheckBox.Value
    ];
end