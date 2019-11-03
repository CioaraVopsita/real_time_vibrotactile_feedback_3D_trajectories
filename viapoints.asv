function [intersection_point] = viapoints(start_point,target_point,m,theta) %start and target column vectors
%m is the fraction of the straight line trajectory where a via point is
%theta is angle of rotation (in radians)
%wanted
syms x y z t
p = [x; y; z];

%Define perpendicular plane
trajectory_vector = target_point - start_point; %column vector
fraction_viapoint = start_point + m*trajectory_vector;
perpendicular_plane = dot(trajectory_vector, (p-fraction_viapoint));


%Rotate vector by 45 degrees
rotation_matrix = [cos(theta) -sin(theta) 0;...
                   sin(theta) cos(theta) 0;...
                   0 0 1];
trajectory_vector_rotated = rotation_matrix*trajectory_vector; %column vector

%Find intersection of rotated vector with the plane
line_rotated_vector = start_point+trajectory_vector_rotated*t;
t_value = double(solve(subs(perpendicular_plane,[x,y,z],line_rotated_vector)));
intersection_point = start_point+trajectory_vector_rotated*t_value;
end