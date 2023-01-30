%% Circle

x = [0.00, -0.05, -0.21, -0.47, -0.81, -1.21, -1.65, -2.12, -2.57, -3.00, -3.37, -3.67, -3.88, -3.99, -3.99, -3.88, -3.67, -3.37, -3.00, -2.57, -2.12, -1.65, -1.21, -0.81, -0.47, -0.21, -0.05, 0.00];
y = [0.00, 0.46, 0.90, 1.29, 1.60, 1.84, 1.97, 2.00, 1.92, 1.73, 1.45, 1.10, 0.68, 0.23, -0.23, -0.68, -1.10, -1.45, -1.73, -1.92, -2.00, -1.97, -1.84, -1.60, -1.29, -0.90, -0.46, -0.00];
psi = [0.00, 0.23, 0.47, 0.70, 0.93, 1.16, 1.40, 1.63, 1.86, 2.09, 2.33, 2.56, 2.79, 3.03, 3.26, 3.49, 3.72, 3.96, 4.19, 4.42, 4.65, 4.89, 5.12, 5.35, 5.59, 5.82, 6.05, 6.28];
cos_psi = cos(psi)
sin_psi = sin(psi)
arrow_length = 0.1;
y_dir = y + arrow_length*cos_psi;
x_dir = x - arrow_length*sin_psi;

%%
figure(1)
plot(x,y,'-')
hold on 
for i = 1:28
    p_0 = [x(i),y(i)];
    p_1 = [x_dir(i), y_dir(i)]
    drawArrow(p_0,p_1)
end

%% Eight

x = [0, -2, -4, -2, 0, 2, 4, 2, 0]*2.5;
y = [0, 2, 0, -2, 0, 2, 0, -2, 0]*2.5;
psi = [0.78, 1.57, 3.14, 4.71, 5.49, 4.71, 3.14, 1.57, 0.78];
cos_psi = cos(psi);
sin_psi = sin(psi);
arrow_length = 0.1;
y_dir = y + arrow_length*cos_psi;
x_dir = x - arrow_length*sin_psi;
%%

figure(2)
plot(x,y,'-')
hold on 
for i = 1:length(x)
    p_0 = [x(i),y(i)];
    p_1 = [x_dir(i), y_dir(i)]
    drawArrow(p_0,p_1)
end


%% Lemniscate

a = 9.5;
t = [0:0.01:6.29];
x = a*cos(t)./(1+(sin(t).^2));
y = a*sin(t).*cos(t)./(1+(sin(t).^2));
figure(3)
plot(x,y)

%% Sample points from the lemniscate and calculate psi
N = 20; %Number of sampling points
i = round(length(x)/N);
x_samp = x(1:i:end);
y_samp = y(1:i:end);

x_samp_1 = x(2:i:end);
y_samp_1 = y(2:i:end);

deltax_samp = x_samp-x_samp_1;
deltay_samp = y_samp-y_samp_1;
psi_samp = atan2(deltax_samp,-deltay_samp);
cos_psi = cos(psi_samp);
sin_psi = sin(psi_samp);
arrow_length = 1;
y_dir = y_samp + arrow_length*cos_psi;
x_dir = x_samp - arrow_length*sin_psi;

figure(4)
plot(x,y)
hold on 
plot(x_samp, y_samp, '*')
for i = 1:length(x_samp)
    p_0 = [x_samp(i),y_samp(i)];
    p_1 = [x_dir(i), y_dir(i)]
    drawArrow(p_0,p_1)
end

mat = [x_samp;y_samp;psi_samp]
s1 = regexprep(num2str(x_samp),'\s+',',')
s2 = regexprep(num2str(y_samp),'\s+',',')
s3 = regexprep(num2str(psi_samp),'\s+',',')

%% Random path
rng(2)
N = 20;
scale = 7;
curvature = 5;
psi = [];
psi = [psi, rand];
x = [];
y = [];
x = [x, 0];
y = [y, 0];

for i = 1:N
    dist = rand*scale;
    x_next = x(i) - dist*sin(psi(i));
    y_next = y(i) + dist*cos(psi(i));
    psi_next = psi(i) + (rand-0.5)*curvature;
    x = [x; x_next];
    y = [y; y_next];
    psi = [psi; psi_next];
end

x = x(1:end);
y = y(1:end);
cos_psi = cos(psi);
sin_psi = sin(psi);
arrow_length = 0.1;
y_dir = y + arrow_length*cos_psi;
x_dir = x - arrow_length*sin_psi;
%%

figure(2)
plot(x,y,'-')
hold on 
for i = 1:length(x)
    p_0 = [x(i),y(i)];
    p_1 = [x_dir(i), y_dir(i)]
    drawArrow(p_0,p_1)
end
s1 = regexprep(num2str(x'),'\s+',',')
s2 = regexprep(num2str(y'),'\s+',',')
s3 = regexprep(num2str(psi'),'\s+',',')

%% Structured flight

x = [ 8.706  8.706  6.318  3.134  0.    -2.04  -5.224 -7.214 -7.612 -9.204];
y = [-4.03  -1.244 -2.04   1.542  0.    -6.02  -4.03  -1.244 -2.438 -1.244];
delta_x = x(2:end) - x(1:end-1);
delta_y = y(2:end) - y(1:end-1);
psi = [atan2(-delta_x,delta_y), 0];
cos_psi = cos(psi);
sin_psi = sin(psi);
arrow_length = 0.1;
y_dir = y + arrow_length*cos_psi;
x_dir = x - arrow_length*sin_psi;

figure(10)
plot(x,y,'-')
hold on 
for i = 1:length(x)
    p_0 = [x(i),y(i)];
    p_1 = [x_dir(i), y_dir(i)]
    drawArrow(p_0,p_1)
end
psi = psi + pi/2
s1 = regexprep(num2str(x),'\s+',',')
s2 = regexprep(num2str(y),'\s+',',')
s3 = regexprep(num2str(psi),'\s+',',')