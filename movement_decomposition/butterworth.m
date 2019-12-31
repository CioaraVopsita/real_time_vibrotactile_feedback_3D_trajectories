cutoff = 50;
samplerate=240;
Wn = cutoff/(samplerate/2);
[b,a]=butter(2,Wn);

posx = filtfilt(b,a,vel_001(:,4));
posy = filtfilt(b,a,vel_001(:,5));
posz = filtfilt(b,a,vel_001(:,6));

rate=1/240;

Gx = gradient(posx, rate);
Gy = gradient(posy, rate);
Gz = gradient(posz, rate);
Gtang = sqrt(Gx.^2+Gy.^2+Gz.^2);

figure(2)
plot(Gtang)

%ask about delay; ask about a target that can connect to separate trials

%apply to positional data!!!!
%filter delay!

%%final goal is to have patients take a robot at home for correction
%this involves predictive powers that for now are beyond the scope
% and hence, cued movements are appropriate
%% test if iterative yields same results;

% vx = vel(:,1);
% vy = vel(:,2);

    
