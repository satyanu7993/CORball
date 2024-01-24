function dsdt=ballspring(i,k1,k2,m1,m2,m3)

global k1 k2 m1 m2 m3 flag
g=-9.8;

t_target=10;
abtol=1e-4;
options1=odeset('events',@eventFunc1,AbsTol=abtol,RelTol=abtol*10);%settings for flight
options2=odeset('events',@eventFunc2,AbsTol=abtol,RelTol=abtol*10);%settings for stick
x2_init=4;
x2=x2_init;
x1=x2;
x3=0;
v1=0;
v2=0;
v3=0;

t_start=0;
t_end=t_target;
flag=0;
if flag==0
    [t,y]=ode45(@flight,[t_start t_end],[x1 v1 x2 v2 x3 v3],options1);%s=[x1 v1 x2 v2];
else
       v_new=(m2*v2+m3*v3)/(m2+m3);
    v2=v_new;
    v3=v_new;
    x2=x3;
    
    [t,y]=ode45(@stuck,[t_start t_end],[x1 v1 x2 v2 x3 v3 ],options2);%s=[x1 v1 x2 v2];
end
s_all=[];
t_all=[];
t_pc=[0,0];
s_all=[s_all;y];
t_all=[t_all;t];
t_final=t_all(end);

t_pc=[t_pc;[t_final,flag]];

while(t_final<t_target)
x1=s_all(end,1);
x2=s_all(end,3);
x3=s_all(end,5);
v1=s_all(end,2);
v2=s_all(end,4);
v3=s_all(end,6);

t_start=t_final;
t_end=t_target;
if flag==0
    [t,y]=ode45(@flight,[t_start t_end],[x1 v1 x2 v2 x3 v3],options1);%s=[x1 v1 x2 v2];
else
    v_new=(m2*v2+m3*v3)/(m2+m3);
    v2=v_new;
    v3=v_new;
 x2=x3;
    
        [t,y]=ode45(@stuck,[t_start t_end],[x1 v1 x2 v2 x3 v3],options2);%s=[x1 v1 x2 v2];
end
s_all=[s_all;y];
t_all=[t_all;t];
t_final=t_all(end);
t_pc=[t_pc;[t_final,flag]];
end



data=[[k1,k2,m1,m2,m3,x1,x2];[t_all,s_all]];
filename=strcat('ballspring',num2str(i),'.txt')
writematrix(data,filename,'WriteMode','overwrite')
%dlmwrite(filename,data,'-append');
filename=strcat('tpc',num2str(i),'.txt')
writematrix(t_pc,filename,'WriteMode','overwrite')
end


function dsdt=flight(t,s)
global k1 k2 m3 m1 m2 flag
g=-9.8;
dsdt=[s(2);-k1/m1*(s(1)-s(3))+g;s(4);-k1/m2*(s(3)-s(1))+g;s(6);g-k2/m3*s(5)];
end


function dsdt=stuck(t,s)
global k1 k2 m1 m2 m3 
g=-9.8;
M=m2+m3;
dsdt=[s(2);-k1/m1*(s(1)-s(3))+g;s(4);k1/M*(s(1)-s(3))-k2/M*s(5)+g;s(6);k1/M*(s(1)-s(3))-k2/M*s(3)+g];
end



function[check,isterminal,direction]=eventFunc1(t,s)
global k1 k2 m3 m1 m2 flag
check=s(3)-s(5);
%what is =0 to trigger event?
isterminal=1;
%should it stop?
direction=-1;
%=+ve to -ve(-1) or -ve to +ve(+1), or dont care?
flag=1;
end


function[check,isterminal,direction]=eventFunc2(t,s)
global k1 k2 m3 m1 m2 flag
g=-9.8;
check=k1*(s(1)-s(3))-k2*s(5);
%what is =0 to trigger event?
isterminal=1;
%should it stop?
direction=-1;
%=+ve to -ve(-1) or -ve to +ve(+1), or dont care?
flag=0;
end
