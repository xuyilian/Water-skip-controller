sample_rate = 200; %hz
sample_time = 1/sample_rate;
t=0*sample_time:sample_time:60;

R=1;
Rz=0.4;
omega= 0.2*2*pi;
trantime =10;
trantime2 =40;
trantime3 =50;

x=t;
y=t;
z=t;

f = @(x,L,k,x_0) L/(1+exp(-k*(x-x_0)));

L=1;
k=2;
x_0=10;
x_1= 50;



for i=1:1:length(t)
   x(i)=R*cos(omega*(t(i)-trantime))* f(t(i),L,k,x_0)*  f(t(i),L,-k,x_1) ;
end

for i=1:1:length(t)
   y(i)=R*sin(omega*(t(i)-trantime))* f(t(i),L,k,x_0)*  f(t(i),L,-k,x_1);
end

for i=1:1:length(t)
    z(i)=Rz*cos(0.5*omega*(t(i)-trantime))* f(t(i),L,k,x_0)*  f(t(i),L,-k,x_1);
end


z=z+0.8;

%% diff
% kernels for convolution
diff_kernel = [0.5, 0, -0.5];
ddiff_kernel = [1, -2, 1];

% position velocity
xd= conv(x, diff_kernel, 'valid')/sample_time;
yd= conv(y, diff_kernel, 'valid')/sample_time;
zd= conv(z, diff_kernel, 'valid')/sample_time;

% position acc
xdd= conv(xd, diff_kernel, 'valid')/sample_time;
ydd= conv(yd, diff_kernel, 'valid')/sample_time;
zdd= conv(zd, diff_kernel, 'valid')/sample_time;

% position acc
xddd= conv(xdd, diff_kernel, 'valid')/sample_time;
yddd= conv(ydd, diff_kernel, 'valid')/sample_time;
zddd= conv(zdd, diff_kernel, 'valid')/sample_time;

% position acc
xdddd= conv(xddd, diff_kernel, 'valid')/sample_time;
ydddd= conv(yddd, diff_kernel, 'valid')/sample_time;
zdddd= conv(zddd, diff_kernel, 'valid')/sample_time;

x=x(1+4:end-4);
y=y(1+4:end-4);
z=z(1+4:end-4);
xd=xd(1+3:end-3);
yd=yd(1+3:end-3);
zd=zd(1+3:end-3);
xdd=xdd(1+2:end-2);
ydd=ydd(1+2:end-2);
zdd=zdd(1+2:end-2);
xddd=xddd(1+1:end-1);
yddd=yddd(1+1:end-1);
zddd=zddd(1+1:end-1);
t=t(1+4:end-4);





%% plot trajectory
figure(1)
plot3(x,y,z)
axis equal

figure(2)
subplot(3,1,1)
plot(t,x)
subplot(3,1,2)
plot(t,y)
subplot(3,1,3)
plot(t,z)

% figure(3)
% subplot(3,1,1)
% plot(t,xd)
% subplot(3,1,2)
% plot(t,yd)
% subplot(3,1,3)
% plot(t,zd)
% 
% 
% figure(4)
% subplot(3,1,1)
% plot(t,xdd)
% subplot(3,1,2)
% plot(t,ydd)
% subplot(3,1,3)
% plot(t,zdd)

%% logestic function

% f = @(x,L,k,x_0) L/(1+exp(-k*(x-x_0)));
% 
% L=1;
% k=2;
% x_0=5;
% x=0:0.1:10;
% for i=1:1:length(x)
%     A(i)=f(x(i),L,k,x_0);
% end
% figure
% plot(x,A)


%% save datas

save('T1.mat',...
    'x','y','z',...
    'xd','yd','zd',...
    'xdd','ydd','zdd',...
    'xddd','yddd','zddd',...
    'xdddd','ydddd','zdddd')
