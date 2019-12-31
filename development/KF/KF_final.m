%% Define var

%Note hann new function in Matlab

%Online version won't have size of t known
%Make computation of noise separate functions and call them here

%System
%Given time vector         20 is way too much!!!!!
T = 30*0.0042;
time = time(1:30:end);
%State Matrix
A = [1 T T^2/2 0 0 0 0 0 0;... state_vector = [x dx ddx y dy ddy z dz ddz]
     0 1 T 0 0 0 0 0 0;...
     0 0 1 0 0 0 0 0 0;...
     0 0 0 1 T T^2/2 0 0 0;... 
     0 0 0 0 1 T 0 0 0;...
     0 0 0 0 0 1 0 0 0;...
     0 0 0 0 0 0 1 T T^2/2;...
     0 0 0 0 0 0 0 1 T;...
     0 0 0 0 0 0 0 0 1];

%Observation matrix
%C = eye(3,9);
C = eye(9);
     

%Process Noise covariance - acceleration or jerk as white noise; multiply
%by jvar

Q = ([T^3/6 T^2/2 T 0 0 0 0 0 0]'*[T^3/6 T^2/2 T 0 0 0 0 0 0]+...
     [0 0 0 T^3/6 T^2/2 T 0 0 0]'*[0 0 0 T^3/6 T^2/2 T 0 0 0]+...
     [0 0 0 0 0 0 T^3/6 T^2/2 T]'*[0 0 0 0 0 0 T^3/6 T^2/2 T])*100;  
 
%Process noise for acceleration as added noise
% w = [T^2/2 T^2/2 T^2/2 T T T 1 1 1]'.*randn(9,1);

% %Process noise for jerk as added white noise
% w = [T^3/6 T^3/6 T^3/6 T^2/2 T^2/2 T^2/2 T T T]'.*randn(9,1);

%Initial position, velocity and acceleration
%x0 = posx(1); y0 = posy(1); z0 = posz(1); % informed estimate
%velx0 = 0; vely0 = 0; velz0 = 0;
%accx0 = 0; accy0 = 0; accz0 = 0;

%True state for offline only
true_state = data(1:30:end,:)'; %size tx3 (posx posy posz); transposed so that it is 3xt
%add_zeros = zeros(6,length(time));
%true_state = [true_state; add_zeros];

%Pre-allocate matrix for measurements (only offline)
z = zeros(9,length(time));
%% Obs

%Observation
for i=1:length(time)
    [v,R] = measurement_noise;
    z(:,i) = C*true_state(:,i)+v;
end

%% 

initial_state = C\z(:,1);

%Estimated state
predicted_state = zeros(9,length(time));
predicted_state(:,1) = initial_state;


%Define I
I = eye(9,9);

%Covariance matrix
predicted_P = zeros(9,9,length(time));
estimated_P = zeros(9,9,length(time));
predicted_P(:,:,1) = Q;

K = zeros(9,9,length(time));

% V = 0.01*ones(1,9);
% predicted_P = diag(V); %9x9

%% KF

%KF
for i=2:length(time) %change this!!!!! MATRICES MADNESS
    
    predicted_state(:,i) = A*predicted_state(:,i-1); %!!!!!!!!!!
    predicted_P(:,:,i) = A*predicted_P(:,:,i-1)*A' + Q;
    
    %Correct
    %Kalman gain INVERSE
    K(:,:,i) = predicted_P(:,:,i)*C'*pinv(C*predicted_P(:,:,i)*C'+R); %Approx of matrix inverse to avoid singularity
    
    %Measurement
    [v,R] = measurement_noise;
    z(:,i) = C*true_state(:,i)+v;
    
    %Update state vector
    predicted_state(:,i) = predicted_state(:,i)+K(:,:,i)*(z(:,i)-C*predicted_state(:,i));
    %Update covariance
    predicted_P(:,:,i) = (I-K(:,:,i)*C)*predicted_P(:,:,i)*(I-K(:,:,i)*C)'+K(:,:,i)*R*K(:,:,i)'; %Joseph form
end

for f = 1:9
%     figure(f)
%     plot(true_state(f,:)); hold on; plot(predicted_state(f,:));
    
    figure(10)
    subplot(3,3,f), plot(true_state(f,:)); hold on; plot(predicted_state(f,:));
end

