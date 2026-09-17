close all

sample_time = 0.01;


figure(1)
plot(Z)
[x,~]=ginput(2);
startpoint = round(x(1));
endpoint = round(x(2));
close 1
%%


quat = [QW;QX;QY;QZ]';
rotm = quat2rotm(quat);
h = mean(diff(Abs_time));

R13 = permute(rotm(1,3,:),[3,2,1]);
R23 = permute(rotm(2,3,:),[3,2,1]);


deltaR = rotm(:,:,end-1);
axang = zeros(length(Abs_time)-1,4);
for i=2:1:length(Abs_time)
    deltaR(:,:,i-1) = rotm(:,:,i-1)\rotm(:,:,i);
    axang(i-1,:) = rotm2axang(deltaR(:,:,i-1));
end

%%
rotation_axis = mean(axang(startpoint:endpoint,1:3))';
R13_new = Abs_time;
R23_new = Abs_time;
for i=1:1:length(Abs_time)
    axis_temp = rotm(:,:,i)*rotation_axis;
    R13_new(i) = axis_temp(1);
    R23_new(i) = axis_temp(2);
end

figure(2)
subplot(2,1,1)
plot(Abs_time,R13); hold on
plot(Abs_time,R13_new); 
xlabel('time (s)')
ylabel('R13')
hold off
subplot(2,1,2)
plot(Abs_time,R23); hold on
plot(Abs_time,R23_new);
xlabel('time (s)')
ylabel('R23')
hold off