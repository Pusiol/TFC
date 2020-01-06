t=0:0.0001:1/2/60;
k=0.001;

s=220*sin(2*pi*60*t);
S=220*cos(2*pi*60*t)*2*pi*60;

hold on
N=k*S-s+400;

d=N./400;

for i=1:length(d)
    if(d(i)>0.95)
        d(i)=0.95;
    end
end

%plot(d)

plot(s/220)


ril1=(1/85e-3).*s.*d*(1/2000);%plot(ril1)
ril2=(1/85e-3).*(400-s).*(1-d)*(1/2000);%plot(ril2)

plot((ril1+ril2)/2)



