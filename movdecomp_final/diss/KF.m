reaching = true;

%System
t = time;
rate = 0.0042;

%State Matrix
F = [1 rate; 0 1];

%Input matrix
G = [-1/2*rate^2; -rate];

%Observation matrix
H = [1 0];

%Noice covariance
Q = [0 0; 0 0];

%action
u = 1;

I = eye(2);

%Initial velocity and acceleration
v0 = 0;
a0 = 0;

true_state = zeros(2,size(t));
true_state(:,1) = [v0; a0];

%Prediction eq %how far ahead??
while reaching
    for i = 2:100000000
        true_state(:,i) = F*true_state(:,i-1) + G*u;
    end
end

%Noisy measurement
v = randn(1,size(t));
z = H*true_state+v;

%Estimated state
estimated_state = zeros(2,size(t));
estimated_state(:,1) = [0; 0];

%Covariance matrix

C = [10 0; 0 0.01];

%KF
for i=2:1000000 %change this!!!!!
    estimated_state(:,i) = F*estimated_state(:,i-1) + G*u;
    %New covariance
    C = F*C*F' + Q;
    %Kalman gain
    K = C*H'/(H*C*H'); %check equations!!! +r
    %Update state vector
    estimated_state(:,i) = estimated_state(:,i)+K*(z(i)-H*estimated_state(:,i));
    %Update covariance
    C = (I-K*H)*C;
end


