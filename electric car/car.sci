//clear console
clc;
clear;
xdel;
// lets make a project folder in C:\Projekt
chdir("C:\"); // posiible we need admin permission in w7/w10
//mkdir("Projekt");
cd("Projekt");
disp(pwd()); // check workspace dir
clear x;clear ans;
 
 
 
 
// define vehicle parameters
// weight, spin resistance, rolling resistance, aerodynamic resistance,face surface,efficiency,recuperation
car_t=struct('m',2500,   'fm',1.05,   'Crr',0.012,   'Cx',0.28,   'A',2.8, 'eff', 0.9, 'rec',0.4);
 
// additional parameters
// wind speed, air density, earth acc
other_t=struct('Vw',0, 'rho',1.2, 'g',9.81);
 
 
 
 
 
// open data file
filename = "nowa-trasa.csv"; // here define name of data file
myfile = csvRead(filename); // read as float
time_str_vect = csvRead(filename,",", ".", 'string');// form strings time vect
 
 
// form all needed vectors
// form speed vector
speed_km_vect=myfile(:,6);
speed_km_vect(1)=[]; // drop first val (%nan)
//disp(speed_km_vect); // show speed vector
 
// form distance vector
distance_vect=myfile(:,7);
distance_vect(1)=[];
//disp(distance_vect);
 
// form altitude vector
altitude_vect=myfile(:,5);
altitude_vect(1)=[];
//disp(altitude_vect);
 
 
// form time vector with maximum enable resolution
time_str_vect(:,1)=[];// drop useless data
time_str_vect(:,2:8)=[];
time_str_vect(1,:)=[];
// from 2017-11-25 11:04:30.168 to 2004 06 10 17 00 12.00
// year montt day hour minutes seconds miliseconds
time_str_vect=strsubst(time_str_vect,'-',' '); // form to getdate() result
time_str_vect=strsubst(time_str_vect,':',' '); // switch ':' char to ' '
time_str_vect=strsubst(time_str_vect,'.',' ');
 
row_cnt = size(time_str_vect,'r'); // get size
cnt =1;
time_vect(1)=[0];
delta_time_vect=[];
 
while cnt < row_cnt
    // parse data
    trash=[]; year=[]; month=[]; day=[];
    hour=[]; minute=[]; second=[]; ms=[];
    // form t1 for etime func
    [trash,year,month,day,hour,minute,second,ms]=msscanf(1,time_str_vect(cnt),"%d %d %d %d %d %d %d %e %e");
    second=second+ms/1000;
    t1=[year month day hour minute second];
    // form t2 for etime func
    [trash,year,month,day,hour,minute,second,ms]=msscanf(1,time_str_vect(cnt+1),"%d %d %d %d %d %d %d %e %e");
    second=second+ms/1000; // wrong separator in format %e for scanf
    t2=[year month day hour minute second];
    time_vect(cnt+1)=time_vect(cnt)+etime(t2,t1);// finaly form time vector  
    delta_time_vect(cnt)=etime(t2,t1); // calculate delta time in s
    cnt=cnt+1;
end
clear row_cnt; clear cnt; clear myfile;
clear second; clear trash; clear year;
clear month; clear day; clear hour;
clear minute; clear ms; clear filename;
clear t1; clear t2; clear time_str_vect;
// and now we have formed only needed vectors form file
   
 
 
// form km/h to m/s
row_cnt=size(speed_km_vect,'r');
cnt=1;
speed_vect=[];
while cnt<=row_cnt
    speed_vect(cnt)=speed_km_vect(cnt)/3.6;
    cnt=cnt+1;
end
clear row_cnt; clear cnt;
   
 
 
 
// lets calculate acceleration vect
// calc right sided derivative (v2-v1)/dt
acceleration_vect =[];
row_cnt=size(speed_vect,'r');
cnt=1;
 
while(cnt<=(row_cnt))
    // to change !!
    if(cnt==row_cnt)then // fix vector size
        acceleration_vect(cnt)=0;
        break;
    end
    // calc derivative
    acceleration_vect(cnt)=(speed_vect(cnt+1)-speed_vect(cnt))/delta_time_vect(cnt);
    cnt=cnt+1;
end
clear row_cnt; clear cnt;
 
 
 
// calc tilt in degrees
tilt_rad_vect=[]; // radians values
tilt_dgr_vect=[]; // degrees values
row_cnt=size(distance_vect,'r');
cnt=1;
while(cnt<row_cnt)
    a =(altitude_vect(cnt+1)-altitude_vect(cnt));
    b =(distance_vect(cnt+1)-distance_vect(cnt))*1000;
    tilt_rad_vect(cnt)=a/b;
    if(tilt_rad_vect(cnt)>1) then // if sinx > 1 abort
        disp("BROKEN ALTITUDE VECT!!!");
        abort;
    end;
    tilt_rad_vect(cnt)=asin(tilt_rad_vect(cnt));
    tilt_dgr_vect(cnt)=(tilt_rad_vect(cnt)*360)/(2*%pi); // convert rad to dgr
    cnt=cnt+1;
end
 
clear row_cnt; clear cnt; clear a; clear b;
 
 
 
 
 
 
// calc used energy
force_vect=[];
power_vect=[];
 
text_buffer =[]; // string to parse displayed data
row_cnt=size(speed_vect,'r');
cnt=1;
 
disp("nr odcinka, faza ruchu, przyspieszenie [m/s^2], kąt nachylenia [stopnie], Fa,Fb,Ft,Fw - poszczególne siły [N], F - łączna siła [N], P -moc [Ws] ");
while cnt<row_cnt
    text_buffer = string(cnt)+" odcinek, ";
    if acceleration_vect(cnt)>0 then // acceleration
       text_buffer=text_buffer+"przyspieszanie, ";
       text_buffer=text_buffer+"a="+string(acceleration_vect(cnt))+" kąt="+string(tilt_dgr_vect(cnt));
       Fb=acceleration_vect(cnt)*car_t.m*car_t.fm// inertial force
       Fa=0.5*((speed_vect(cnt)-other_t.Vw)).^2*other_t.rho*car_t.A*car_t.Cx;// air resistance
       Fw=car_t.m*other_t.g*sin(tilt_rad_vect(cnt));    //
       Ft=car_t.Crr*car_t.m*other_t.g*cos(tilt_rad_vect(cnt));
       force_vect(cnt)=Fa+Fb+Fw+Ft; //
       text_buffer=text_buffer+" Fa="+string(Fa)+" Fb="+string(Fb)+" Fw="+string(Fw)+" Ft="+string(Ft);
       text_buffer=text_buffer+" F="+string(force_vect(cnt));
       power_vect(cnt)=force_vect(cnt)*speed_vect(cnt+1); // calculate power  
       text_buffer=text_buffer+" P="+string(power_vect(cnt));
    elseif acceleration_vect(cnt)<0 then // braking
       text_buffer=text_buffer+"hamowanie,      ";
       text_buffer=text_buffer+"a="+string(acceleration_vect(cnt))+" kat="+string(tilt_dgr_vect(cnt));
       Fb=acceleration_vect(cnt)*car_t.m*car_t.fm;
       Fa=0.5*((speed_vect(cnt)-other_t.Vw)).^2*other_t.rho*car_t.A*car_t.Cx;  
       Fw=car_t.m*other_t.g*sin(tilt_rad_vect(cnt));  
       Ft=car_t.Crr*car_t.m*other_t.g*cos(tilt_rad_vect(cnt));
       force_vect(cnt)=Fa+Fw+Ft+Fb;
       text_buffer=text_buffer+" Fa="+string(Fa)+" Fb="+string(Fb)+" Fw="+string(Fw)+" Ft="+string(Ft);
       text_buffer=text_buffer+" F="+string(force_vect(cnt));
       power_vect(cnt)=force_vect(cnt)*speed_vect(cnt+1);
       text_buffer=text_buffer+" P="+string(power_vect(cnt));
    else // constant speed
       text_buffer=text_buffer+"stala predkosc, ";
       text_buffer=text_buffer+"a="+string(acceleration_vect(cnt))+" kat="+string(tilt_dgr_vect(cnt));
       Fa=0.5*((speed_vect(cnt)-other_t.Vw)).^2*other_t.rho*car_t.A*car_t.Cx;// V^2*rho*A*Cx  
       Fw=car_t.m*other_t.g*sin(tilt_rad_vect(cnt)); // m*g*sin(alfa in radians)
       Ft=car_t.Crr*car_t.m*other_t.g*cos(tilt_rad_vect(cnt));
       force_vect(cnt)=Fa+Fw+Ft;
       text_buffer=text_buffer+" Fa="+string(Fa)+" Fw="+string(Fw)+" Ft="+string(Ft);
       text_buffer=text_buffer+" F="+string(force_vect(cnt));
       power_vect(cnt)=force_vect(cnt)*speed_vect(cnt);
       text_buffer=text_buffer+" P="+string(power_vect(cnt));
    end
    // display info about every part
    disp(text_buffer);
    cnt=cnt+1;
end
disp(" "); // new line
disp(" "); // new line
disp(" "); // new line
clear row_cnt; clear cnt; clear Fa; clear Fb; clear Fw; clear Ft; clear text_buffer;
 
 
//power having regard to the efficiency of the power unit
row_cnt=size(power_vect,'r');
cnt=1;
while cnt<=row_cnt
    if power_vect(cnt)>0
        power_vect(cnt)=power_vect(cnt)/0.9; //90% efficiency power unit
    else
        power_vect(cnt)=power_vect(cnt)*0.9; // in recuperation mode
    end
    cnt=cnt+1;
end
clear row_cnt; clear cnt;
 
 
// hardcode fix vectoer size to calculate work and draw scopes/plots
power_vect=cat(1,power_vect,0); //  add 0 on end of vector
tilt_dgr_vect=cat(1,tilt_dgr_vect,0); //  add 0 on end of vector
force_vect=cat(1,force_vect,0); //  add 0 on end of vector
tilt_rad_vect=cat(1,tilt_rad_vect,0); //  add 0 on end of vector
 
 
 
// calc power consumption for 100% recuperation, rec val recuperation and 0%
work =[];
work = inttrap(time_vect,power_vect); // calc integral of power in time
work=work/3600;// result was in W/s so convert to W/h
disp("Zuzyta energia przy 100% odzysku - " + string(work)+"W/h");
 
row_cnt=size(power_vect,'r');
cnt=1;
P_rec_vect=[]; // temporary vector for acc val
P_zero_vect=[]; // temporary vector for 0%
while cnt<=row_cnt
    if (power_vect(cnt)>0) then
        P_rec_vect(cnt)=power_vect(cnt);
        P_zero_vect(cnt)=power_vect(cnt);
    else
        P_rec_vect(cnt)=power_vect(cnt)*car_t.rec;
        P_zero_vect(cnt)=0;
    end
    cnt=cnt+1;
end
clear row_cnt; clear cnt;
 
work_rec = inttrap(time_vect,P_rec_vect);
work_rec=work_rec/3600;// result was in W/s so convert to W/h
work_zero = inttrap(time_vect,P_zero_vect);
work_zero=work_zero/3600;// result was in W/s so convert to W/h
disp("Zuzyta energia przy zdefiniowanym odzysku - " + string(work_rec)+"W/h");
disp("Zuzyta energia przy braku odzysku - " + string(work_zero)+"W/h");
 
 
 
 
show_plots = 1;// define 0 - hide plots, 1-show plots  
 
if(show_plots~=0) then
    //1
    scf(1);
    subplot(211);
    plot2d(time_vect,speed_km_vect);
    xtitle('V=f(t)', 't, [s]', 'V, [km/h]')
    subplot(212);
    plot2d(time_vect,distance_vect);
    xtitle('l=f(t)', 't, [s]', 'l, [km]')  
    //2
    scf(2);
    subplot(211);
    plot2d(time_vect,acceleration_vect);
    xtitle('a=f(t)', 't, [s]', 'a, [m/s^2]');
    subplot(212);
    plot2d(time_vect,power_vect);
    xtitle('P=f(t)', 't, [s]', 'P, [W]')
    //3
    scf(3)
    subplot(211);
    plot2d(time_vect,force_vect);
    xtitle('F=f(t)', 't, [s]', 'F, [N]')  
    subplot(212);
    plot2d(time_vect,power_vect);
    xtitle('P=f(t)', 't, [s]', 'P, [W]')
    //4
    scf(4)
    subplot(111);
    plot2d(time_vect,tilt_dgr_vect);
    xtitle('altitude=f(t)', 't, [s]', 'tilt, [degree]')
end
 
 
 
 
 
 
// battery aproximation func (overvoltage! soc(1)=4,33V)
function w=e(soc)// lion
 w = n*(-1.031*exp(-35*soc)+3.685+0.2156*soc+0.1178*soc*soc+0.3201*soc*soc*soc);
endfunction
 
// better battery aproximation func, but still its charging function (soc(1)~4,1V)
// source: State of charge estimation for Li-ion battery based on extended Kalman filter Li Zhia, Zhang Pengb
//, Wang Zhifua , Song Qianga, Rong Yinana
function w=e_new(soc)// lion
 w = n*( -3.836*soc*soc*soc*soc + 10.32*soc*soc*soc-9.671*soc*soc + 4.162*soc+3.19);
endfunction
 
 
n = 84; // in row 84s50p
 
Cn = 160; // capacity in Ah
Cns = Cn * 3200;
Rw = n*(0.025/Cn) // 25mohm
Pmax = e(1)^2/(4*Rw);
 
vector_length = size(time_vect,'r');
 
 
 
// calculate power consumptrion with defined recuperation
j =1;
soc_rec(j) = 1.0;
q(j) = Cns*soc_rec(j);
 
 
while e_new(soc_rec(j))^2-4*Rw*P_rec_vect(j) >= 0
   
    i_rec(j) = (e_new(soc_rec(j))-sqrt(e_new(soc_rec(j))^2-4*Rw*P_rec_vect(j)))/(2*Rw);
    u_rec(j) = e_new(soc_rec(j))-i_rec(j)*Rw;
   
    p_rec(j) = i_rec(j)*u_rec(j);
   
    //q(j+1) = q(j) - i_rec(j) * delta_time_vect(j); //euler
    // huen ------------
    q_new(j) = q(j) - i_rec(j) * delta_time_vect(j);
    soc_new(j) = q_new(j) / Cns;
    i_new(j) = (e_new(soc_new(j))-sqrt(e_new(soc_new(j))^2-4*Rw*P_rec_vect(j)))/(2*Rw);
    q(j+1) = q(j)-(i_rec(j) + i_new(j)) * delta_time_vect(j)/2;
    //----------------
   
   
    soc_rec(j+1) = q(j+1) / Cns; // soc value
    if soc_rec(j) >1  then soc_rec(j)=1 end
   
    if u_rec(j) <= n*2.9 then break;end //break
    j = j + 1;  // inc j
    if(j>size(delta_time_vect,'r')) then break end;
end
 
 
 
// fix vector size
i_rec(vector_length)=0;
p(vector_length)=0;
u_rec(vector_length)=u_rec(vector_length-1);
 
 
scf(5);
subplot(211);
plot2d(time_vect, i_rec);
xtitle('i(t) (z rekuperacja)','t, s', 'i, A');
subplot(212);
plot2d(time_vect,tilt_dgr_vect);
xtitle('altitude=f(t)', 't, [s]', 'tilt, [degree]');
 
scf(6)
subplot(211);
plot2d(time_vect, u_rec);
xtitle('u(t) (z rekuperacja)','t, s', 'u, V');
subplot(212);
plot2d(time_vect, soc_rec);
xtitle('SoC(t) (z rekuperacja)','t, s', 'SoC');
 
disp('-----------------------------------------------------------------------------');
disp('Zuzycie energii akumulatora na badanym odcinku z rekuperacją');
disp('stan naładowania akumulatora po przejechaniu trasy z rekuperacją '+string(car_t.rec*100)+'% SOC='+string(soc_rec(vector_length)));
disp('napiecie poczatkowe U_pocz='+string(u_rec(1))+'V, napiecie koncowe U_konc='+string(u_rec(vector_length))+'V');
 
 
 
 
 
// without zerouperation
j =1;
soc_zero(j) = 1.0;
q(j) = Cns*soc_zero(j);
 
 
while e_new(soc_zero(j))^2-4*Rw*P_zero_vect(j) >= 0
   
    i_zero(j) = (e_new(soc_zero(j))-sqrt(e_new(soc_zero(j))^2-4*Rw*P_zero_vect(j)))/(2*Rw);
    u_zero(j) = e_new(soc_zero(j))-i_zero(j)*Rw;
   
    p_zero(j) = i_zero(j)*u_zero(j);
   
    //q(j+1) = q(j) - i_zero(j) * delta_time_vect(j); //euler
    // huen ------------
    q_new(j) = q(j) - i_zero(j) * delta_time_vect(j);
    soc_new(j) = q_new(j) / Cns;
    i_new(j) = (e_new(soc_new(j))-sqrt(e_new(soc_new(j))^2-4*Rw*P_zero_vect(j)))/(2*Rw);
    q(j+1) = q(j)-(i_zero(j) + i_new(j)) * delta_time_vect(j)/2;
    //----------------
   
   
    soc_zero(j+1) = q(j+1) / Cns; // soc value
    if soc_zero(j) >1  then soc_zero(j)=1 end
   
    if u_zero(j) <= n*2.9 then break;end //break
    j = j + 1;  // inc j
    if(j>size(delta_time_vect,'r')) then break end;
end
 
 
 
// fix vector size
i_zero(vector_length)=0;
p(vector_length)=0;
u_zero(vector_length)=u_zero(vector_length-1);
 
 
scf(7);
subplot(211);
plot2d(time_vect, i_zero);
xtitle('i(t) (bez rekuperacji)','t, s', 'i, A');
subplot(212);
plot2d(time_vect,tilt_dgr_vect);
xtitle('altitude=f(t)', 't, [s]', 'tilt, [degree]');
 
scf(8)
subplot(211);
plot2d(time_vect, u_zero);
xtitle('u(t) (bez rekuperacji)','t, s', 'u, V');
subplot(212);
plot2d(time_vect, soc_zero);
xtitle('SoC(t) (bez rekuperacji)','t, s', 'SoC');
 
 
 
 
 
 
 
disp('-----------------------------------------------------------------------------');
disp('Zuzycie energii akumulatora na badanym odcinku bez rekuperacji');
disp('stan naładowania akumulatora po przejechaniu trasy SOC='+string(soc_zero(vector_length)));
disp('napiecie poczatkowe U_pocz='+string(u_zero(1))+'V, napiecie koncowe U_konc='+string(u_zero(vector_length))+'V');
disp('success');