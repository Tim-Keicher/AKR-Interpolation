%function berechnet aus vm, bm und se die Zeiten tb, tv und te
function [ta, tv, te]=calc_t_ramp(se, vm, am)
syms t
for i=1:size(vm,1)
    if se(i)>0 %Abfangen von negativen Eingaben bei Wegstrecke
        if vm(i)>(se(i)*am(i))^0.5 %Prüfung ob vm höher ist als vm,max
        vm(i)=(se(i)*am(i))^0.5; %vm auf vm,max setzen
        disp('Programmierte Geschwindigkeit wird nicht erreicht')
        end
        ta(i)=vm(i)/am(i);
        te(i)=se(i)/vm(i)+ta(i);
        tv(i)=te(i)-ta(i);
    else
        ta(i)=0;te(i)=0;tv(i)=0; %Nullsetzen der Ausgabewerte im Fehlerfall
    end
end

 