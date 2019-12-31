%liberty_realtime_demo
%
%simple graphing of on-line collection
%using the Prok.liberty interface to the liberty
%
%Chris Miall 21/8/2018

fprintf('-------------------------------------------------------\n');
fprintf('8 channel Liberty realtime demo\n');
fprintf('-------------------------------------------------------\n');
clear;
close all;
fclose all;
% Open active x control
fprintf('Wait for library to install ..');
h = actxserver('Prok.Liberty');
%start up data transfer
h.connect;
fprintf('..done\n');
sens=h.sensors; % how many are connnected?
rate = 240;
sample_interval = 1/rate; %going at 240 Hz
markers = 0;
while min(markers) <1 || max(markers) >sens,
    markerlist = getstring(['Select markers (eg 1 2 ... max ' num2str(sens) ' or RETURN)'],'');
    markers = [];
    if isempty(markerlist),
        markers = 1:sens;
    else
        while ~isempty(markerlist),
            [m,markerlist] = strtok(markerlist,' ,;:/-');
            markers = [markers str2num(m)];
        end
        markers = sort(markers);
        sens=length(markers);
    end
end
fprintf('Using markers %d %d %d %d %d %d %d %d\n',markers);
%block unwanted channels
sensor_array = zeros(8,1);
sensor_array(markers)=1;
h.sensor_map = sensor_array;
h.hemisphere=[1 0 0 ];
KbName('UnifyKeyNames');
escapeKey = KbName('ESCAPE');
%% loop around
% hFig = figure();
% hold on;
% set(hFig,'WindowStyle','docked');
% hTxt=text(40,-40,-40,'waiting for data');
% hDat=plot3(NaN,NaN,NaN,'+','MarkerSize',14);
% set(gca,'xlim',[0 50],'ylim',[-50 50],'zlim',[-50 50],'Zdir','reverse');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');

%% !!! PREALLOCATION
step=10; %step 50 
data={}; n=1;
data{n} = [];
position{n} = {};
velocity{n} = {};
velocity_tangential{n} = {};
Vtang{n} = {};
PeakStruct{n} = {};

number_trials = 51;

%Velocity iterables
j = 2;
i = 1;
m = 1;
PeakTable = [];
error_sound = false;
vibration = false;
sample=0;

%KF initialisation
T = 0.0042;

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
 
%Observation matrix - uses positional data alone
 C = [1 0 0 0 0 0 0 0 0;...
     0 0 0 1 0 0 0 0 0;...
     0 0 0 0 0 0 1 0 0];
 
R = [0.02 0 0;...
     0 0.025 0;...
     0 0 0.01]*10^(-4.5);
 
Rvel = [0.01 0 0;...
        0 0.012 0;...
        0 0 0.005]*10^(-7); 
 
%Process Noise covariance - acceleration or jerk as white noise; multiply
%by jvar
Q = ([T^3/6 T^2/2 T 0 0 0 0 0 0]'*[T^3/6 T^2/2 T 0 0 0 0 0 0]+...
     [0 0 0 T^3/6 T^2/2 T 0 0 0]'*[0 0 0 T^3/6 T^2/2 T 0 0 0]+...
     [0 0 0 0 0 0 T^3/6 T^2/2 T]'*[0 0 0 0 0 0 T^3/6 T^2/2 T])*100; 

Qvel = [T^4/4 0 0;...
        0 T^4/4 0;...
        0 0 T^4/4];

predicted_state = [];
predicted_position = [];
predicted_velocity = [];
predicted_velocity_tangential = [];

predicted_P = Q;
predicted_Pvel = Qvel;
I = eye(9,9);
Ivel = eye(3,3);

%Peak detection
AmpThreshold = 20;
PeakTable = [];
PeakNumber=0;
WidthThreshold = 1;

ioObj = io32; %do not run this twice, without clearing it in between
status = io32(ioObj); %should be zero
io32address = hex2dec('1008');  %non-standard LPT1 output port address


start_sound = audioplayer(sin(1:1500),8000);
end_sound = audioplayer(tan(1:1500),7000);

%!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
target_point = [107.67775; 1.79784; 19.18348];
intersection_point = [71.933790005949790;-31.898487025208420;-22.137627174269102];
elbow_room = zeros(3,number_trials);
max_elbow_room = 0.3*(target_point-intersection_point);
min_elbow_room = [4; 4; 4];
elbow_room(:,1:3) = repmat(max_elbow_room,1,3);
intersection_range = [intersection_point-max_elbow_room intersection_point+max_elbow_room];
endpoint_range = [target_point-1 target_point+1];
success_history = zeros(1,number_trials);
recorded_history = false;


%axis equal; keep t0=tic close to toc(t0)
h.start;

t0=tic;

%% CORE

while n<number_trials, %until figure is closed
    tmp = h.position;
    timestamp = toc(t0);
    
    %fprintf('x y x %d %d %d\n', tmp([1,2,3]));
    if ~isempty(tmp)
        if timestamp<3 %&& not at target yet
            data{n} = [data{n}; timestamp tmp];
            trial_data = data{n};
            
            %mark on the floor polhemus and participant spot.
            %kalman filter for position for n motion trackers => n
            %condition 1 only 1 motion marker position x cm from the
            %target (x always the same).
            
            %z for one marker only
            z = tmp(1:3)';
            
            %for multiple motion trackersl PCToolbox is needed. For
            %now, leave it like this because I only need one motion
            %tracker.
            
            if vibration
                if toc(start_vibration)>0.15
                    io32(ioObj,io32address,0);
                    vibration=false;
                end
            end
            
            %start bleep - take into account the 20 timesteps
            if i==20 %tic toc here and for vibration!!!!!!
                play(start_sound);  
                fprintf('\n Start');
            end
            
            
            if i==1
                predicted_state(:,1) = C\z;
                predicted_position(:,1) = predicted_state([1,4,7],1);
                predicted_velocity = zeros(3,1);
                predicted_velocity_tangential(1) = 0;
                i = i+1;
                
            else
            
                predicted_state(:,i) = A*predicted_state(:,i-1);
                predicted_P = A*predicted_P*A' + Q;

                %Correct
                %Kalman gain
                K = (predicted_P*C'*pinv(C*predicted_P*C'+R)); %Approx of matrix inverse to avoid singularity

                %Update state vector
                predicted_state(:,i) = predicted_state(:,i)+K*(z-C*predicted_state(:,i));

                %Update covariance
                predicted_P = (I-K*C)*predicted_P*(I-K*C)'+K*R*K'; %Joseph form
                
                predicted_position(:,i) = predicted_state([1,4,7],i); 
                
                if i>=step
                    %position_kf = predicted_state([1,4,7],:);
                    velocity_kf = mean(gradient(predicted_position(:,i-(step-1):i),1/rate),2);
                    
                    %SDKF
                    predicted_velocity(:,j) = predicted_velocity(:,j-1);
                    predicted_Pvel = predicted_Pvel + Qvel;
                    
                    Kvel = predicted_Pvel*pinv(predicted_Pvel+Rvel); 
                  
                    predicted_velocity(:,j) = predicted_velocity(:,j)+Kvel*(velocity_kf-predicted_velocity(:,j));
    
                    predicted_Pvel = (Ivel-Kvel)*predicted_Pvel;
                    
                    predicted_velocity_tangential(j) = norm(predicted_velocity(:,j));
                    
                    if j>=9
                       smoothing_vector = N(4.5,4,[1:1:9]);
                       temp_signal = conv2(predicted_velocity_tangential((j-8):j),smoothing_vector,'same');
                       Vtang_smoothed(m) = temp_signal(5);
                       if m>2 % Start peak detection when 3 points have been acquired.
                           if Vtang_smoothed(m-1)>AmpThreshold % If a point is greater than the amplitude threshold set in line 23
                               if (Vtang_smoothed(m-1)>Vtang_smoothed(m-2) && Vtang_smoothed(m-1)>Vtang_smoothed(m)) % AND greater than the following one, register a peak.
                                   if isempty(PeakTable)
                                       [Width] = gaussfit([(m-2) (m-1) (m)],[Vtang_smoothed(m-2) Vtang_smoothed(m-1) Vtang_smoothed(m)]);
                                       if Width>WidthThreshold
                                           Time_Peak = timestamp; Amplitude_Peak = Vtang_smoothed(m-1); Index = m-1;
                                           PeakNumber=PeakNumber+1;
                                           PeakTable(PeakNumber,:)=[PeakNumber,Time_Peak,Amplitude_Peak,Index,Width]; %struct+reset peaknumber+change gaussfit to one output
                                       end
                                   else
                                       if (m-1)-PeakTable(end,4) > 40
                                           [Width] = gaussfit([(m-2) (m-1) (m)],[Vtang_smoothed(m-2) Vtang_smoothed(m-1) Vtang_smoothed(m)]);
                                           if Width>WidthThreshold
                                               Time_Peak = timestamp; Amplitude_Peak = Vtang_smoothed(m-1); Index = m-1;
                                               PeakNumber=PeakNumber+1;
                                               PeakTable(PeakNumber,:)=[PeakNumber,Time_Peak,Amplitude_Peak,Index,Width];
                                           end
                                       else
                                           change_interpeak = Vtang_smoothed(PeakTable(end,4):m-1);
                                           valley = min(change_interpeak);
                                           if Vtang_smoothed(PeakTable(end,4))-valley>1/10*Vtang_smoothed(PeakTable(end,4)) && Vtang_smoothed(m-1)-valley>1/10*Vtang_smoothed(m-1)
                                               [Width] = gaussfit([(m-2) (m-1) (m)],[Vtang_smoothed(m-2) Vtang_smoothed(m-1) Vtang_smoothed(m)]);
                                               if Width>WidthThreshold
                                                   Time_Peak = timestamp; Amplitude_Peak = Vtang_smoothed(m-1); Index = m-1;
                                                   PeakNumber=PeakNumber+1;
                                                   PeakTable(PeakNumber,:)=[PeakNumber,Time_Peak,Amplitude_Peak,Index,Width];
                                               end
                                           end
                                       end
                                   end
                               end
                           end
                       end
                       m = m+1;
                    end
                    j = j+1;
                end
                i = i+1;
            end
            
            %Start giving feedback
            %Positive Feedback on position
            if i>20
                if predicted_position(:,end)>intersection_range(:,1) & predicted_position(:,end)<intersection_range(:,2)
                        start_vibration_viaPoint = tic;
                        io32(ioObj,io32address,'01000000');
                        %fprintf('vb on');
                        vibration = true;
                        if recorded_history == false
                            success_history(n) = 1;
                            recorded_history = true;
                        end
                end
                
                if predicted_position(:,end)>endpoint_range(:,1) & predicted_position(:,end)<endpoint_range(:,2)
                    start_vibration_endPoint = tic;
                    io32(ioObj,io32address,'01000000');
                end
            end
            
            %!!!!!!!!!!!!!add feedback for target reaching
            
            %Positive feedback on velocity
            
            
            
            sample=sample+sample_interval;
            while toc(t0)<=sample %replace with toc(t0)
            end
            
        elseif timestamp>3 && timestamp<5 %&& not at target yet
        %if not at target think how you do this so that you do not
        %check if you reached the target every timestamp following time
        %overshoot
            
            
            if vibration
                if toc(start_vibration_viaPoint)>0.15
                    io32(ioObj,io32address,0);
                    vibration=false;
                end
            end
            
            
            %Psychometric Staircase
            if n>2
                if success_history(n)==0 && success_history(n-1)==0 && success_history(n-2)==0
                    elbow_room_change = 1.1*elbow_room(:,n);
                    if all(elbow_room_change<=max_elbow_room)
                        elbow_room(:,n+1) = elbow_room_change;
                    else
                        elbow_room(:,n+1) = max_elbow_room;
                    end
                    intersection_range = [intersection_point-elbow_room(:,n+1) intersection_point+elbow_room(:,n+1)];
                elseif success_history(n)==1 && success_history(n-1)==1 && success_history(n-2)==1
                    elbow_room_change = 0.8*elbow_room(:,n);
                    if all(elbow_room_change>=min_elbow_room)
                        elbow_room(:,n+1) = elbow_room_change;
                    else
                        elbow_room(:,n+1) = min_elbow_room;
                    end
                    intersection_range = [intersection_point-elbow_room(:,n+1) intersection_point+elbow_room(:,n+1)];
                else
                    elbow_room(:,n+1) = elbow_room(:,n);
                    intersection_range = [intersection_point-elbow_room(:,n+1) intersection_point+elbow_room(:,n+1)];
                end
            end
            
        
            if error_sound==false
              play(end_sound);
              fprintf('\n ERROR');
              error_sound=true;
            end

            position{n} = predicted_position;
            velocity{n} = predicted_velocity;
            velocity_tangential{n} = predicted_velocity_tangential;
            Vtang{n} = Vtang_smoothed;
            PeakStruct{n} = PeakTable;
        
        %else
        %end
        %tmp = zeros(1,6*length(markers));
        elseif timestamp>5
            error_sound=false;
            recorded_history = false;
            sample = 0;
            PeakNumber = 0;
            PeakTable = [];
            i = 1;
            j = 2;
            m = 1;
            n = n+1;
            data{n} = [];
            t0 = tic; 
        end
    
%     set(hTxt,'String',sprintf('Data found at time %d',timestamp)); %replace with toc(t0)
%     tmp1=reshape(tmp,6,sens)';
%     set(hDat,'xdata',tmp1(:,1),'ydata',tmp1(:,2),'zdata',tmp1(:,3));
%     drawnow();
    end
%     sample=sample+sample_interval;
%     while timestamp<=sample %replace with toc(t0)
%     end
end




h.stop;
% Disconnect
h.disconnect;
%fprintf('\n Approx sample rate was %f per second\n',1/mean(diff(data(:,1))));