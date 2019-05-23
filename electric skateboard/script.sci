//The MIT License

//Copyright (c) Pawel L. @miszczo 2019 

//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in
//all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.


// Created by Pawel L. - miszczo
// based on http://jaroszynski.pollub.pl/dydaktyka/

clear;
clc;
xdel(winsid());

// dane pojazdu i otoczenia

debug = 0;
data_t=struct('a_acc',0.6, 'a_brake',2, 'Vm',10, 'm',100, 'fm',1, 'Crr',0.05, 'Cx',0.9, 'A',2, 'Vw',0, 'rho',1.2, 'g',9.81);

distance_v={110;1000;6500;1400;2500;900;1500;2000;2400;2000;900};// dlugosci poszczegolnych odcinkow

tilt_dgr_v={0.224;-0.1644;-0.0263;-0.0053;0.093;1.063;-0.0176;-0.1126;0.0029;-0.1848;-0.2858}//nachylenie

seg_nr=11;// zdefiniowana ilosc odcinkow
precision_step = 10;// krok obliczen
vect_long = seg_nr*precision_step*3; // dlugosc powstalych wektorow
// szukane wektory
v_vect={};
P_vect={};
t_vect={};
W_vect={};

//dzielenie odcinków na poszczegolne fazy ruchu, 
for i = 1:seg_nr 

    t_acc=data_t.Vm/data_t.a_acc;// czas osciagniecia v_max

    t_brake=data_t.Vm/data_t.a_brake;// czas wyhamwoania

    new_distance_v(i,1)=data_t.a_acc*t_acc*t_acc/2;// droga przejechana podczas rozpedzania
    new_distance_v(i,3)=data_t.a_brake*t_brake*t_brake/2;
    new_distance_v(i,2)=distance_v(i)-new_distance_v(i,1)-new_distance_v(i,3);
    new_distance_v(i,4)=tilt_dgr_v(i)*%pi/180;// wpisanie kąta(TODO: zmienic na rad?)
    // wyswietl parametry ruchu
    t_const=new_distance_v(i,2)/data_t.Vm;// czas jazdy ze stala predkoscia
    disp('Odcinek nr: '+string(i)+' s');
    disp('Czas rozpedzania: '+ string(t_acc)+' s');
    disp('Droga rozpedzania: '+ string(new_distance_v(i,1))+' m');
    disp('Czas jazdy z v=const: '+ string(t_const)+' s');
    disp('Droga jazdy z v=const: '+ string(new_distance_v(i,2))+' m');
    disp('Czas hamowania: '+ string(t_brake)+' s');
    disp('Droga hamowania: '+ string(new_distance_v(i,3))+' m');
    disp('-----------------------------------------------------------');
    if i==seg_nr then 
        distance_v=new_distance_v;
        clear new_distance_v;clear t_acc;clear t_brake;
        clear i; clear tilt_dgr_v;
    end
end
//---------------------------------------------


// liczymy szukane wektory t,p,v
for i=1:seg_nr//
    // ----------------------------------faza przyspieszania--------------------
    _V=linspace(0,data_t.Vm,precision_step);// wektor predksoci chwilowej
    Fb = data_t.a_acc*data_t.m*data_t.fm;// sila bezwladnosci 
    Fa = 0.5*((_V-data_t.Vw)).^2*data_t.rho*data_t.A*data_t.Cx;  //opor aerodynamcizny
    Fw = data_t.m*data_t.g*sin(distance_v(i,4));  // itd
    Ft = data_t.Crr*data_t.m*data_t.g*cos(distance_v(i,4));
    F=Fa+Fw+Ft+Fb;
    P=F.*_V/1000;   
 
    
    v_vect={v_vect,_V};//OGOLNY WEKTOR PREDKSOCI
    P_vect={P_vect,P}; //OGOLNY WEKTOR MOCY
    
    t_acc=data_t.Vm/data_t.a_acc;
    _t1=0;
    if i>1 then _t1=t_vect((i-1)*precision_step*3);end
    _t2=_t1+t_acc;
    _temp=linspace(_t1,_t2,precision_step);
    t_vect={t_vect,_temp};  // !!!!OGOLNY WEKTOR CZASU
    // czyszczenie zmienncyh
    clear Fa;clear Fb;clear Fw; clear Ft; clear F;
    clear P; clear t_acc;clear _t1;clear _t2;clear temp;
    clear _V;clear _temp;
    // -------------------------------------------------------------------------
    
    // ----------------------------faza ruchu jednostajnego --------------------
	Fa = 0.5*((data_t.Vm-data_t.Vw))^2*data_t.rho*data_t.A*data_t.Cx;  
    Fw = data_t.m*data_t.g*sin(distance_v(i,4));
    Ft = data_t.Crr*data_t.m*data_t.g*cos(distance_v(i,4));
    F=Fa+Fw+Ft;
    P=F*data_t.Vm/1000;  
    
    _V=linspace(data_t.Vm,data_t.Vm,precision_step);
    _P=linspace(P,P,precision_step);
    v_vect={v_vect,_V};//OGOLNY WEKTOR PREDKSOCI
    P_vect={P_vect,_P}; //OGOLNY WEKTOR MOCY
    
    
    t_const=distance_v(i,2)/data_t.Vm;
    _t1=t_vect((i-1)*3*precision_step+10);//????/??
    _t2=_t1+t_const;
    _temp=linspace(_t1,_t2,precision_step);
    t_vect={t_vect,_temp};
    clear Fa;clear Fb;clear Fw; clear Ft; clear F;
    clear P; clear t_const;clear _t1;clear _t2;clear temp;
    clear _V;clear _temp;clear _P;
    // ------------------------------------------------------------------------

    // ----------------------------------faza hamowania------------------------
    _V=linspace(data_t.Vm,0,precision_step);// wektor predksoci chwilowej
    Fb = -data_t.a_brake*data_t.m*data_t.fm;// sila bezwladnosci 
    Fa = 0.5*((_V-data_t.Vw)).^2*data_t.rho*data_t.A*data_t.Cx;  //opor aerodynamcizny
    Fw = data_t.m*data_t.g*sin(distance_v(i,4));  // itd
    Ft = data_t.Crr*data_t.m*data_t.g*cos(distance_v(i,4));
    F=Fa+Fw+Ft+Fb;
    P=F.*_V/1000;   
    
    v_vect={v_vect,_V};//OGOLNY WEKTOR PREDKSOCI
    P_vect={P_vect,P}; //OGOLNY WEKTOR MOCY

    t_brake=data_t.Vm/data_t.a_brake;
    _t1=t_vect((i-1)*3*precision_step+20);
    _t2=_t1+t_brake;
    _temp=linspace(_t1,_t2,precision_step);
    t_vect={t_vect,_temp};  // !!!!OGOLNY WEKTOR CZASU
    
    
    // czyszczenie zmienncyh
    clear Fa;clear Fb;clear Fw; clear Ft; clear F;
    clear P; clear t_brake;clear _t1;clear _t2;clear temp;
    clear _V;clear _temp;
    // ------------------------------------------------------------------------
    
  if i==seg_nr then clear i;end 
end


// liczymy zuzycie energii na poszczegolnych odcinkach

for i=1:seg_nr
    if debug>0 then
        //disp(t_vect(i*10-9));\
        disp(i);
        disp('---------------');
        disp((i-1)*30+1);
        disp((i-1)*30+10);
    
        disp('---------------');
        disp((i-1)*30+11);
        disp((i-1)*30+21);
        disp('---------------');
        disp((i-1)*30+21);
        disp((i-1)*30+30);
    end
    
    disp('Zuzycie energii na odcinku nr: '+string(i));
    W_vect(i,1)=inttrap(t_vect((i-1)*30+1:i*30),P_vect((i-1)*30+1:i*30))/3600;
    W_vect(i,2)=inttrap(t_vect(((i-1)*30+11):((i-1)*30+20)),P_vect(((i-1)*30+11):((i-1)*30+20)))/3600;
    W_vect(i,3)=inttrap(t_vect(((i-1)*30+21):((i-1)*30+30)),P_vect(((i-1)*30+21):((i-1)*30+30)))/3600;
    disp('I faza '+string(W_vect(i,1))+'kWh  II faza '+string(W_vect(i,2))+'kWh  III faza '+string(W_vect(i,3))+' kWh');
end

W_summary=inttrap(t_vect,P_vect)/3600;
if debug>0 then
    disp(W_vect);
    disp(P_vect)
end
// wykreslamy wykresy p(t) oraz v(t)
if debug <1 then
    scf();
    subplot (1,1,1);
    plot2d(t_vect,P_vect);
    xtitle('wykres P = f(t)','czas, s','moc, kW')
    legend('Moc');

    scf();
    subplot (1,1,1);
    plot2d(t_vect,v_vect);
    xtitle('wykresP v = f(t)','czas, s','predkosc, m/s')
    legend('Predkosc');
end


function w=e(soc)// dla liion
 w = n*(-1.031*exp(-35*soc)+3.685+0.2156*soc+0.1178*soc*soc+0.3201*soc*soc*soc)
endfunction


n = 10; // liczba ogniw szeregowych
Cn = 27; // pojemnosć w Ah
Cns = Cn * 3600; // pojemnosć w kulombach
Rw = n*(0.06/Cn) // rezystancja wew
Pmax = e(1)^2/(4*Rw); //moc maksymalna w stanie dopasowania


disp('--------------------Parametry baterii----------------------');
disp('Pojemnosc Cn='+string(Cn)+' Ah');
disp('Rezystancja wew. Rw='+string(Rw*1000)+' mohm');
disp('Moc maksylana Pmax='+string(Pmax)+' W');
disp('-----------------------------------------------------------');


//---------------------Z REKUPERACJA-----------------------//
j =1;
soc(j) = 1.0; // początkowy stan naładowania
q(j) = Cns*soc(j); // ładunek początkowy

//t_vect(331)=t_vect(330)+1;
while e(soc(j))^2-4*Rw*P_vect(j)*1000 >= 0
    if j<vect_long then // ma to sens tylko wtedy, gdy ostatnia faza ruchu konczy sie zatrzymaniem,
        dt=t_vect(j+1)-t_vect(j);   // wyznaczmy dt
        if P_vect(j+1)<0 then P_vect(j+1)=P_vect(j+1)*0.3; end end // brak rekuperacji, wektor mocy zostaje zniszczony
    i(j) = (e(soc(j))-sqrt(e(soc(j))^2-4*Rw*P_vect(j)*1000))/(2*Rw); //obliczony prąd   
    u(j) = e(soc(j))-i(j)*Rw; //obliczone napięcie
    u(j)=u(j)*0.853;// poprawka na prawidłowe napiecie dla ogniw liion
    p(j) = i(j)*u(j); //obliczona moc
    q(j+1) = q(j) - i(j) * dt; //ładunek
    soc(j+1) = q(j+1) / Cns; // stan naładowania
    
    if u(j) <= n*2.9 then break;end// przerwij jesli bateia rozladowana
    j = j + 1;  // inc j
    if j==vect_long+1 then break; end// koniec trasy
end
if debug <1 then
    scf; plot2d(t_vect(1:vect_long), i); xtitle('i(t) (z rekuperacja)','t, s', 'i, A');
    scf; plot2d(t_vect(1:vect_long), u); xtitle('u(t) (z rekuperacja)','t, s', 'u, V');
    scf; plot2d(t_vect(1:vect_long), soc(1:vect_long)); xtitle('SoC(t) (z rekuperacja)','t, s', 'SoC');
end

disp('stan naładowania akumulatora po przejechaniu trasy z rekuperacją SOC= '+string(soc(vect_long)));


//---------------------BEZ REKUPERACJI-----------------------//
j =1;
soc(j) = 1.0; // początkowy stan naładowania
q(j) = Cns*soc(j); // ładunek początkowy

//t_vect(331)=t_vect(330)+1;
while e(soc(j))^2-4*Rw*P_vect(j)*1000 >= 0
    if j<vect_long then // ma to sens tylko wtedy, gdy ostatnia faza ruchu konczy sie zatrzymaniem,
        dt=t_vect(j+1)-t_vect(j);   // wyznaczmy dt
        if P_vect(j+1)<0 then P_vect(j+1)=0; end end // brak rekuperacji, wektor mocy zostaje zniszczony
    i(j) = (e(soc(j))-sqrt(e(soc(j))^2-4*Rw*P_vect(j)*1000))/(2*Rw); //obliczony prąd   
    u(j) = e(soc(j))-i(j)*Rw; //obliczone napięcie
    u(j)=u(j)*0.853;// poprawka na prawidłowe napiecie dla ogniw liion
    p(j) = i(j)*u(j); //obliczona moc
    q(j+1) = q(j) - i(j) * dt; //ładunek
    soc(j+1) = q(j+1) / Cns; // stan naładowania
   
   
    if u(j) <= n*2.9 then break;end// przerwij jesli bateia rozladowana
    j = j + 1;  // inc j
    if j==vect_long+1 then break; end// koniec trasy
end

if debug <1 then
    scf; plot2d(t_vect(1:vect_long), i); xtitle('i(t) (bez rekuperacji)','t, s', 'i, A');
    scf; plot2d(t_vect(1:vect_long), u); xtitle('u(t) (bez rekuperacji)','t, s', 'u, V');
    scf; plot2d(t_vect(1:vect_long), soc(1:vect_long)); xtitle('SoC(t) (bez rekuperacji)','t, s', 'SoC');
end
disp('stan naładowania akumulatora po przejechaniu trasy bez rekuperacji SOC= '+string(soc(vect_long)));

