%function berechnet aus vm, bm und se die Zeiten tb, tv und te (V07 S24)
function [tb, tv, te]=calc_t_ramp(vm, bm, se)
if se>=0 %Abfangen von negativen Eingaben bei Wegstrecke
    if vm>sqrt(se*bm)  %Prüfung ob vm höher ist als vm,max
        vm=sqrt(se*bm); %vm auf vm,max setzen
        disp('Programmierte Geschwindigkeit wird nicht erreicht')
    end
    %Berechne Zeiten
    tb=vm/bm;    %Beschleunigungszeit
    if (vm+tb)==0
        disp('Zaehler darf nicht null sein!')
        te=0;
        tv=0;
    else
        te=se/vm+tb; %Verfahrenszeit
        tv=te-tb;    %Verzoegerungszeit
    end
else
    disp('se muss positiv sein!')%Fehlermeldung 
    tb=0;te=0;tv=0; %Nullsetzen der Ausgabewerte im Fehlerfall
end

 