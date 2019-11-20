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

%% Variables that can change

%How many via-points
no_viapoints = 2;
ideal_intervals = [85 175 155];
intersection_point(:,1) = [53.200063736453640;-3.066039158092281;-16.244148728378400];
intersection_point(:,2) = [94.121535660032780;-45.512774165010170;7.839798070657082];

number_trials = 51;
velocity_threshold = 11;
position_threshold = 5;
%Peaks leeway
bound = 15;

%% Preallocation and initialisation of variables

%Structures initialisation
step=10;
data={}; n=1;
data{n} = [];
position{n} = {};
velocity{n} = {};
velocity_tangential{n} = {};
Vtang{n} = {};
PeakStruct{n} = {};

%Velocity iterables
j = 2;
i = 1;
m = 1;
PeakTable = [];
error_sound = false;
feedback_played=false;
vibration = false;
velocity_vibration = false;
threshold_variable = false;
movement_start = false;
sample=0;
Index = 0;

%% Kalman Filter
%KF initialisation
T = 0.0042;

%State Matrix
a = [1 T T^2/2;  %state_vector = [x dx ddx y dy ddy z dz ddz]
     0 1 T;
     0 0 1];
A = blkdiag(a,a,a);

%Observation matrix - uses positional data alone
c = [1 0 0];
C = blkdiag(c,c,c); 
 
%Measurement noise covariance matrix for KF
R = diag([0.02, 0.025, 0.01])*10^(-4.5);

%Measurementnoise covariance matrix for SDKF
Rvel = diag([0.01, 0.012, 0.005])*10^(-7);

%Process Noise covariance matrix for KF - jerk as white noise; 
q = [T^6/36 T^5/12 T^4/6; 
     T^5/12 T^4/4 T^3/2;
     T^4/6 T^3/2 T^2];
Q = blkdiag(q,q,q)*100; 

%Process Noise covariance matrix for SDKF - jerk as white noise;  
Qvel = T^4/4 * eye(3); 

predicted_state = [];
predicted_position = [];
Vtang_smoothed = [];
predicted_velocity = [];
predicted_velocity_tangential = [];

predicted_P = Q;
predicted_Pvel = Qvel;
I = eye(9);
Ivel = eye(3);

%% Variables for velocity peak detection
AmpThreshold = 20;
PeakTable = [];
PeakNumber=0;
WidthThreshold = 1;

%% Parallel port initialisation 
%Parallel port object
ioObj = io32; %do not run this twice, without clearing it in between
status = io32(ioObj); %should be zero
io32address = hex2dec('1008');  %non-standard LPT1 output port address
data_out = '00000000';
data_out_cellarray = {strcat(strrep(data_out(1),'0','1'),data_out(2:end)), strcat(strrep(data_out(1),'1','0'),data_out(2:end));...
                      strcat(data_out(1:4),strrep(data_out(5),'0','1'),data_out(6:end)), strcat(data_out(1:4),strrep(data_out(5),'1','0'),data_out(6:end))};

%% Sound initialisation
%Sounds to signal start and end of trial
start_sound = audioplayer(sin(1:1500),8000);
end_sound = audioplayer(tan(1:1500),7000);
feedback_sound = load('train');
feedback_sound.y = feedback_sound.y(1:9000);
feedback_sound.Fs = 9000;

%% Viapoints variables
%Via-points computation; initialisation of psychometric staircase values
target_point = [105; 1; 21];

max_elbow_room = [10;10;10];
min_elbow_room = [4; 4; 4];
elbow_room = zeros(3,no_viapoints,number_trials);
elbow_room(:,:,1:5) = repmat(max_elbow_room,[1 no_viapoints 5]);

intersection_range = zeros(3,2,no_viapoints);
for p = 1:no_viapoints
    intersection_range(:,:,p) = [intersection_point(:,p)-max_elbow_room intersection_point(:,p)+max_elbow_room];
end

%Padded with an extra row of zeros to be the same size as
%success_history_velocity for ease of providing parallel feedback witht the
%2 actuators
success_history = zeros(no_viapoints+1,number_trials,2,2);  
peak_time_range = zeros(no_viapoints+1,2,number_trials);
position_history = zeros(1,no_viapoints);
velocity_history = zeros(1,no_viapoints+1);

%Start data collection
h.start;

%Start timing
t0=tic;

%% CORE - Providing feedback

while n<number_trials, %until the set number of trials is completed
    tmp = h.position;
    timestamp = toc(t0);
    
    if ~isempty(tmp)
        %For interval of data collection when participants are prompted to move
        if timestamp<3 
            data{n} = [data{n}; timestamp tmp];
            trial_data = data{n};

            %z for one marker only
            z = tmp(1:3)';
            
            %for multiple motion trackersl PCToolbox is needed. For
            %now, leave it like this because I only need one motion
            %tracker.
            
            %Check if positional feedback is being given and stop it after
            %150ms
%             if vibration && ...
%                toc(start_vibration)>0.1
%                     data_out = strcat(strrep(data_out(1),'1','0'),data_out(2:end));
%                     io32(ioObj,io32address,bin2dec(data_out));
%                     vibration=false;
%             end
            
            %Check if velocity feedback is being given and stop it after
            %150ms
%             if velocity_vibration && ...
%                toc(start_vibration_velocity)>0.1
%                     data_out = strcat(data_out(1:4),strrep(data_out(5),'1','0'),data_out(6:end));
%                     io32(ioObj,io32address,data_out);
%             end
            
            %Wait 20 timestamps before instructing participants to move to
            %allow KF to converge
            if i==20 
                play(start_sound);  
                fprintf('\n Start');
            end
            
            
            if i==1 %Initial parameters for KF
                predicted_state(:,1) = C\z;
                predicted_position(:,1) = predicted_state([1,4,7],1);
                predicted_velocity = zeros(3,1);
                predicted_velocity_tangential(1) = 0;
                initial_position = z;
                start_range = [initial_position-position_threshold initial_position+position_threshold];
                i = i+1;
                
            else
                
                %Prediction
                predicted_state(:,i) = A*predicted_state(:,i-1);
                predicted_P = A*predicted_P*A' + Q;

                %Correction
                %Kalman gain
                K = (predicted_P*C'*pinv(C*predicted_P*C'+R)); %Approx of matrix inverse to avoid singularity

                %Update state vector
                predicted_state(:,i) = predicted_state(:,i)+K*(z-C*predicted_state(:,i));

                %Update covariance
                predicted_P = (I-K*C)*predicted_P*(I-K*C)'+K*R*K'; %Joseph form
                
                predicted_position(:,i) = predicted_state([1,4,7],i); 
                
                if i>=step %Start SDKF
                    
                    velocity_kf = mean(gradient(predicted_position(:,i-(step-1):i),1/rate),2);
                    
                    %SDKF prediction
                    predicted_velocity(:,j) = predicted_velocity(:,j-1);
                    predicted_Pvel = predicted_Pvel + Qvel;
                    
                    %SDKF correction
                    %Kalman gain
                    Kvel = predicted_Pvel*pinv(predicted_Pvel+Rvel); 
                    
                    %Update state vector
                    predicted_velocity(:,j) = predicted_velocity(:,j)+Kvel*(velocity_kf-predicted_velocity(:,j));
    
                    predicted_Pvel = (Ivel-Kvel)*predicted_Pvel;
                    
                    predicted_velocity_tangential(j) = norm(predicted_velocity(:,j));
                    
                    if j>=9 %further smooth velocity
                       smoothing_vector = N(4.5,4,[1:1:9]);
                       temp_signal = conv2(predicted_velocity_tangential((j-8):j),smoothing_vector,'same');
                       Vtang_smoothed(m) = temp_signal(5);
                       
                       if m>2 && ... % Start peak detection when 3 points have been acquired.
                               Vtang_smoothed(m-1)>AmpThreshold && ... % If a point is greater than the amplitude threshold
                               (Vtang_smoothed(m-1)>Vtang_smoothed(m-2) && Vtang_smoothed(m-1)>Vtang_smoothed(m)) % AND greater than the following one, register a peak.
                           
                           %Compute the width of the peak to eliminate high frequency noise (potentially caused by interference or other factors)
                           [Width] = gaussfit([(m-2) (m-1) (m)],[Vtang_smoothed(m-2) Vtang_smoothed(m-1) Vtang_smoothed(m)]);
                           
                           %If this is the first peak of the trial..
                           if isempty(PeakTable) && ...
                                   Width>WidthThreshold %If true peak, register it in table
                               Amplitude_Peak = Vtang_smoothed(m-1); Index = m-1;
                               PeakNumber=PeakNumber+1;
                               PeakTable(PeakNumber,:)=[PeakNumber,Amplitude_Peak,Index,Width];
                               
                               %If other peaks have been registered before, check the distance between the 2 potential peaks
                           else
                               %If the distance betweem the two peaks is greater than 40 timestamps
                               if (m-1)-PeakTable(end,3) > 40 && ...
                                       Width>WidthThreshold %If true peak, register it in table
                                   Amplitude_Peak = Vtang_smoothed(m-1); Index = m-1;
                                   PeakNumber=PeakNumber+1;
                                   PeakTable(PeakNumber,:)=[PeakNumber,Amplitude_Peak,Index,Width];
                                   
                                   %If the distance betweem the two peaks is less than 40 timestamps
                               elseif (m-1)-PeakTable(end,3) < 40 && ...
                                       Width>WidthThreshold
                                   %Check the change in velocity between the two peaks
                                   change_interpeak = Vtang_smoothed(PeakTable(end,3):m-1);
                                   valley = min(change_interpeak);
                                   if Vtang_smoothed(PeakTable(end,3))-valley>1/10*Vtang_smoothed(PeakTable(end,3)) && Vtang_smoothed(m-1)-valley>1/10*Vtang_smoothed(m-1)
                                       %Compute the width of the peak to eliminate high frequency noise (potentially caused by interference or other factors)
                                       Amplitude_Peak = Vtang_smoothed(m-1); Index = m-1;
                                       PeakNumber=PeakNumber+1;
                                       PeakTable(PeakNumber,:)=[PeakNumber,Amplitude_Peak,Index,Width];
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
            
            %Positive feedback on position
            %Feedback if the via-point (+/- the allowed spatial interval) is reached
            %Feedback if target (+/- the allowed spatial interval) is reached
            if i>20
                for p=1:no_viapoints
                    if (all(predicted_position(:,end)>intersection_range(:,1,p)) & all(predicted_position(:,end)<intersection_range(:,2,p))) && ...
                        position_history(p) == false
                            success_history(p,n,:,1) = [1, timestamp];
                            position_history(p) = true;
                            %start_vibration = tic;
                            %data_out = strcat(strrep(data_out(1),'0','1'),data_out(2:end));
                            %io32(ioObj,io32address,bin2dec(data_out)); %start port 7
                            %vibration = true;
                    end
                end
                
                %Check if the reaching movement has started based on positional and velocity thresholds. 
                %If movement has started, calculate the time interval within which the peak should occur.
                if movement_start == false && ...
                   (any(predicted_position(:,end) < start_range(:,1)) | any(predicted_position(:,end) > start_range(:,2))) && ...
                    Vtang_smoothed(end)>velocity_threshold
                             start_index = length(Vtang_smoothed) - find(Vtang_smoothed(end:-1:1)<velocity_threshold,1) + 1;
                             if isempty(start_index)
                             else
                                 for p=1:no_viapoints+1
                                     peak_time_range(p,:,n) = [start_index+sum(ideal_intervals(1:p))-bound start_index+sum(ideal_intervals(1:p))+bound];
                                     threshold_variable = true;
                                     movement_start = true;
                                 end
                             end
                end
                
                %Positive feedback on velocity
                %If spatial and velocity feedback have been reached, check
                %if current timestep has peak and if so whether the peak is
                %within the allowed range
                if threshold_variable == true
                    for p = 1:no_viapoints+1 %number of velocity peaks
                            if Index>=peak_time_range(p,1,n) && Index<=peak_time_range(p,2,n) && velocity_history(p) == false
                                    %start_vibration_velocity = tic;
                                    %data_out = strcat(data_out(1:4),strrep(data_out(5),'0','1'),data_out(6:end));
                                    %io32(ioObj,io32address,bin2dec(data_out)); %start port 3
                                    %velocity_vibration = true;
                                    success_history(p,n,:,2) = [1 timestamp];
                                    velocity_history(p) = true;
                            end
                    end
                end
                
            end    
            Index = 0;
            
            %Wait for 0.0042 ms to elapse before checking for the next
            %sample
            sample=sample+sample_interval;
            while toc(t0)<=sample
                
            end
            
        elseif timestamp>=3 && timestamp<5 %1 or 2 seconds?
            
            %Check if positional feedback is being given and stop it after
            %150ms
%             if vibration && ...
%                toc(start_vibration)>0.1
%                     data_out = strcat(strrep(data_out(1),'1','0'),data_out(2:end));
%                     io32(ioObj,io32address,data_out);
%                     vibration=false;
%             end
            
            %Check if velocity feedback is being given and stop it after
            %150ms
%             if velocity_vibration && ...
%                toc(start_vibration_velocity)>0.1
%                     data_out = strcat(data_out(1:4),strrep(data_out(5),'1','0'),data_out(6:end));
%                     io32(ioObj,io32address,data_out);
%                     velocity_vibration=false;
%             end
            
            
            %Signal end of trial
            if error_sound==false
              play(end_sound);
              fprintf('\n ERROR');
              error_sound=true;
            end
            
            
            %Psychometric Staircase
            if n>4
                %Increase spatial feedback range when the via point is missed 3
                %consecutive times
                for p = 1:no_viapoints
                    if all(success_history(p,n-2:n,1,1)==0)
                        elbow_room_change = 1.05*elbow_room(:,p,n);
                        %Limit for increasing spatial feedback range
                        if all(elbow_room_change<=max_elbow_room)
                            elbow_room(:,p,n+1) = elbow_room_change;
                        else
                            elbow_room(:,p,n+1) = max_elbow_room;
                        end
                        intersection_range(:,:,p) = [intersection_point(:,p)-elbow_room(:,p,n+1) intersection_point(:,p)+elbow_room(:,p,n+1)];
                    %Decrease spatial feedback range when the via point is hit
                    %5 consecutive times
                    elseif all(success_history(p,n-4:n,1,1)==1)
                        elbow_room_change = 0.9*elbow_room(:,p,n);
                        %Limit for decreasing spatial feedback range
                        if all(elbow_room_change>=min_elbow_room)
                            elbow_room(:,p,n+1) = elbow_room_change;
                        else
                            elbow_room(:,p,n+1) = min_elbow_room;
                        end
                        intersection_range(:,:,p) = [intersection_point(:,p)-elbow_room(:,p,n+1) intersection_point(:,p)+elbow_room(:,p,n+1)];
                    else
                        elbow_room(:,p,n+1) = elbow_room(:,p,n);
                        %intersection_range(:,:,p) = [intersection_point()-elbow_room(:,n+1) intersection_point+elbow_room(:,n+1)];
                        intersection_range(:,:,p) = intersection_range(:,:,p);
                    end
                end
            end
            position{n} = predicted_position;
            velocity{n} = predicted_velocity;
            velocity_tangential{n} = predicted_velocity_tangential;
            Vtang{n} = Vtang_smoothed;
            PeakStruct{n} = PeakTable;
            
        elseif timestamp>=5 && timestamp<8 % from 5 to 6; train; to 10; from 8 to 9 end of replay sound and wait for next trial
            %Provide offline feedback
            
            %Play sound signalling feedback CHANGE
            if feedback_played==false
              sound(feedback_sound.y,feedback_sound.Fs);
              feedback_played=true;
            end
            
            for measure = 1:2
                if vibration(measure) && toc(uint64(start_vibration(measure)))>=0.1
                    data_out = data_out_cellarray(measure,2);
                    io32(ioObj,io32address,bin2dec(data_out));
                end
            end
            
            if all(move_tonext_index)
                fb_index = fb_index+1;
                fb_given = 0;
                move_tonext_index(:)=0;
            end
            
            %start_vibration = ones(1,2) and RESRET start_vibration(1,2)
            %SAME for start_silence
            %D vibration (1 and 2) silence(1,2)
            
            for measure = 1:2
                if success_history(fb_index,n,1,measure) == 1 && ...
                        abs(success_history(fb_index,n,2,measure)-(timestamp-5))<0.005 &&...
                        move_tonext_index(measure)== 0
                    start_vibration(measure) = tic;
                    data_out = data_out_cellarray(measure,1); %turn vibration on
                    io32(ioObj,io32address,bin2dec(data_out));
                    vibration(measure) = 1;
                    move_tonext_index(measure) = 1;
                elseif success_history(fb_index,n,1,measure) == 0 &&...
                        move_tonext_index(measure)== 0
                    move_tonext_index(measure) = 1;
                end
            end
                
        elseif timestamp>=8
            %Reset all variables for next trial
            error_sound=false;
            feedback_played=false;
            position_history(:) = false;
            velocity_history(:) = false;
            threshold_variable = false;
            movement_start = false;
            predicted_position = [];
            Vtang_smoothed = [];
            Index = 0;
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
    end
end




h.stop;
% Disconnect
h.disconnect;