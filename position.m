close all

quat = [QW;QX;QY;QZ]';
rotm = quat2rotm(quat);
h = mean(diff(Abs_time));

% deltaP=[0.05;-0.0;0];
% L = curveLength(deltaP,X,Y,Z,Abs_time,rotm);

startpoint = 1800;

fun = @(deltaP)curveLength(deltaP,X(800:4000),Y(800:end),Z(800:4000),Abs_time(800:4000),rotm(:,:,800:4000));
x0 = [0;0;0];
options = optimset('PlotFcns',@optimplotfval);

results = fminsearch(fun,x0,options);


%% plot

X_mod = X;
Y_mod = Y;
Z_mod = Z;
for i=1:1:length(Abs_time)
    temp_p = rotm(:,:,i)*results;
    X_mod(i) = X(i)+temp_p(1);
    Y_mod(i) = Y(i)+temp_p(2);
    Z_mod(i) = Z(i)+temp_p(3);
end

figure(2)
plot3(X,Y,Z);hold on
plot3(X_mod,Y_mod,Z_mod)
axis equal
hold off



%% functions
function L = curveLength(deltaP,X,Y,Z,Abs_time,rotm)

X_mod = X;
Y_mod = Y;
Z_mod = Z;
for i=1:1:length(Abs_time)
    temp_p = rotm(:,:,i)*deltaP;
    X_mod(i) = X(i)+temp_p(1);
    Y_mod(i) = Y(i)+temp_p(2);
    Z_mod(i) = Z(i)+temp_p(3);
end
L = 0;
for i=2:1:length(Abs_time)
    L = L + sqrt((X_mod(i) - X_mod(i-1))^2 + (Y_mod(i) - Y_mod(i-1))^2 + (Z_mod(i) - Z_mod(i-1))^2);
end
return;
end