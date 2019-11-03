mean_x = mean(data(:,4));
mean_y = mean(data(:,5));
mean_z = mean(data(:,6));

start_point = [mean_x; mean_y; mean_z];

target_point = [105; 1; 21];

%% One via point


m = 3/10;

theta = -pi/4;

[intersection_point] = viapoints(start_point,target_point,m,theta);

%% Two via points

m1 = 3.3/10;
m2 = 6.6/10;

theta1 = pi/4;
theta2 = -pi/4;

[intersection_point1] = viapoints(start_point,target_point,m1,theta1);
[intersection_point2] = viapoints(start_point,target_point,m2,theta2);
