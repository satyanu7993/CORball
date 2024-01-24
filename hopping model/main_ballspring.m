global k1 k2 m1 m2 m3 flag
plist=[0.01,0.1,1,10,100];%list of parameters
n=length(plist);
i=0;
m1=.1;
m2=m1*1;
m3=m1*.10;
n=7;
k1l=logspace(-2,5,n);
k2l=logspace(-2,5,n);
%k1=10;
for i1=1:n
    for i2=1:n;
    k2=k1l(i1);
    k1=k2l(i2);
    i=i+1;
    
    ballspring(i,k1,k2,m1,m2,m3)
    end
end

