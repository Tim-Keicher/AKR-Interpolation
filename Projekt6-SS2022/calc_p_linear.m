function [p,v,a, glg_a, glg_v,glg_s,te,glg_a_p,glg_v_p,glg_s_p,Zeit]=calc_p_linear(p_start,p_end,vm,am,t_start)
syms t  real
dir=sign(p_end-p_start); % direction -> Bewegungsrichtung wird bestimmt 

% Berechnung der jeweiligen Streckenlängen in x und y Richtung 
if (p_end(2,1)-p_start(2,1))==0 % Unterscheidung zwischen Achsbewegungsrichtung
    s_ep= [sqrt((p_end(1,1)-p_start(1,1))^2+(p_end(2,1)-p_start(2,1))^2);0]; % x-Richtung
else 
    s_ep= [0;sqrt((p_end(1,1)-p_start(1,1))^2+(p_end(2,1)-p_start(2,1))^2)]; % y-Richtung
end 

% Berechnung der Zeiten für a, v, te mit der calc_t_ramp funktion bekannt
% aus Übungsaufgabe
[ta, tv, te]=calc_t_ramp(s_ep, vm, am); % Funktion berechnet aus vm, bm und se die Zeiten tb, tv und te

% Berechnen von Pulsfunktionen "ein oder aus" zum jeweiligen Zeitpunkt für Beschleunigung, konstante Geschwindigkeit und Verzögerung (Heaviside-Funktion)
% Durchlauf für beide Gelenkbeschleunigungen (J1 und J2) - oberen 3 nur für plot
for i=1:size(am) 
    pulse_0a_p(i)=heaviside(t-t_start)-heaviside(t-ta(i)-t_start); %Rechteckimpuls Beschleunigungsphase von 0 nach v
    pulse_av_p(i)=heaviside(t-ta(i)-t_start)-heaviside(t-tv(i)-t_start); %Rechteckimpuls konstante Geschwindigkeit v
    pulse_ve_p(i)=heaviside(t-tv(i)-t_start)-heaviside(t-te(i)-t_start); %Rechteckimpuls Verzögerungsphase von v nach 0
    
    pulse_0a(i)=heaviside(t)-heaviside(t-ta(i)); %Rechteckimpuls Beschleunigungsphase von 0 nach v
    pulse_av(i)=heaviside(t-ta(i))-heaviside(t-tv(i)); %Rechteckimpuls konstante Geschwindigkeit v
    pulse_ve(i)=heaviside(t-tv(i))-heaviside(t-te(i)); %Rechteckimpuls Verzögerungsphase von v nach 0
end

% Berechnung der Gleichungen für s, v ,a - obere 3 nur für plot 
glg_a_p=dir.*am.*pulse_0a_p'-dir.*am.*pulse_ve_p'; %Gleichungen Beschleunigung richtung*Beschleunigung des Gelenks* Wann beschleunigt es und wann nicht
glg_v_p=int(glg_a_p,t); %Gleichungen Geschwindigkeit
glg_s_p=int(glg_v_p,t)+p_start;%Gleichungen Strecke

glg_a=dir.*am.*pulse_0a'-dir.*am.*pulse_ve'; %Gleichungen Beschleunigung richtung*Beschleunigung des Gelenks* Wann beschleunigt es und wann nicht
glg_v=int(glg_a,t); %Gleichungen Geschwindigkeit
glg_s=int(glg_v,t)+p_start;%Gleichungen Strecke

%Bestimmmung der x und y Koordinaten für den TCP - von t0 nach te
%Schrittweise in Abhängigkeit von te, Koordinaten von x und y zum Zeitpunkt t werden in Matrix p erfasst 
% --> egal ob strecke lang oder kurz, Abstände sind näherungsweise immer gleich Lang
te=sum(te,'all'); % Umwandlung von Matrix in einfache Zahl 
NumPoints=(round(te)); 
p=zeros(NumPoints,2);   %  Array mit nur Nullen drin
v=zeros(NumPoints,2);
a=zeros(NumPoints,2);
Zeit=zeros(NumPoints,1);
for i=1:NumPoints   % Positionsbestimmung des TCP  - Alle Werte in eine Positions Matrix,Geschwindigkeitsmatrix und Beschleunigungsmatrix geschrieben 
    tm=(te/(NumPoints-1))*(i-1);
    tmp = subs(glg_s, t, tm);
    tmp_v = subs(glg_v, t, tm);
    tmp_a = subs(glg_a, t, tm);
    p(i,1) = tmp(1);
    p(i,2) = tmp(2);
    v(i,1) = tmp_v(1);
    v(i,2) = tmp_v(2);
    a(i,1) = tmp_a(1);
    a(i,2) = tmp_a(2);
    Zeit(i)=tm+t_start;
end
