%liberty_trial_wrapper_v4
%
%multi-trial data collection trial wrapper
%using the Prok.liberty interface to the liberty
%
%v3 allows interactive control with online, not offline, data collection
%and random time go cues
%
%v4 to interact with the new 8-channel system
%
%Chris Miall 14/8/2012
%final mods 9/11/12:
%%display filename & trial number on screen
%%restart session without liberty start up
%%opt out of random delay
%
%Chris: Snd function used here blocks reading of the data!!! this needs to
%be fixed. 10 July 2019
fprintf('-------------------------------------------------------\n');
fprintf('8 channel Liberty trial wrapper with random time go cue\n');
fprintf('-------------------------------------------------------\n');
% % if getanswer('Have you just started up the Liberty? (say NO if already running!)',1),
% %     fprintf('Waiting for self test\n');
% %     pause(10)
% %     fprintf('Self test complete\n');
% % end
clear;
close all;
fclose all;
col='brgkmycb'; %use for color control
% Open active x control
fprintf('Wait 10 secs for library install ..');
h = actxserver('Prok.Liberty');
%start up data transfer
h.connect;
fprintf('..done\n');
sens=h.sensors; % how many are connnected?

KbName('UnifyKeyNames');
escapeKey = KbName('ESCAPE');
%% loop around
figure_handle = figure;
set(figure_handle,'WindowStyle','docked');
keep_going = 1;
while keep_going,
    %get file name
    [datafilename, pathname] = uiputfile('*_001.dat','Choose the first data filename');
    % this assumes filenames have the form "FRED_002.dat" or "Fred_002" where only the
    % "FRED" bit is critical, and the "_002.dat" will be added autimatically.
    % Don't use the underscore or fullstop characters anywhere else

    %interval to read Liberty data (data appears to be collected at 256 Hz - use tic and toc below to check)
    rate = 240;
    sample_interval = 1/rate; %going at 240 Hz
    repeats = 1;
    % % % repeats = round(240/rate); %number of records per sample
    % % %
    trial_duration = getnumber('Enter sampling period (seconds)', 1.0); %default 1.0 second duration
    if getanswer('Use random delays?',0)
        min_random_duration = getnumber('Enter min. cue period (seconds)', 0.25); %default .25 second min duration
        max_random_duration = getnumber('Enter max. cue period (seconds)', 0.75); %default .75 second max duration
        while max_random_duration <= min_random_duration,
            max_random_duration = getnumber('Re-enter max. cue period (must be greater than min.)', min_random_duration); %default .75 second max duration
        end
    else
        min_random_duration = 0;
        max_random_duration = 0;
    end
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
        end
    end
    fprintf('Using markers %d %d %d %d %d %d %d %d\n',markers);
    records = repmat(markers,repeats,1);
    %block unwanted channels
    sensor_array = zeros(8,1);
    sensor_array(markers)=1;
    h.sensor_map = sensor_array;
    h.hemisphere=[1 0 0 ];
    h.start;
    trial = 1;

%% Repeat until break
    while getanswer('...ready for next trial (''N'' to exit)?',1),

        while KbCheck; end % Wait until all keys are released.
        fprintf('Trial %d ...collecting...', trial);
        Snd('Play',MakeBeep(100,0.05)/6);
        Snd('Play',MakeBeep(300,0.05)/4);
        Snd('Play',MakeBeep(600,0.1)/2);

        %clear space for the data
        data = zeros(ceil((trial_duration + min_random_duration + max_random_duration)/sample_interval),size(markers,2)*6+2+1);
        go = 0;
        wait_duration = min_random_duration + rand*(max_random_duration-min_random_duration);
        starttime=GetSecs;
        t0=tic;
        tic;
        sampletime=0;
        abort = false;
        sample = 1;
        %flush the buffer
        h.flush;
        while ~abort && ((GetSecs-starttime) < ...
                (trial_duration + min_random_duration + max_random_duration))
            % Check the state of the keyboard.
            [ keyIsDown, tmp, keyCode ] = KbCheck;
            % If the user is pressing a key, then display its code number and name.
            if keyIsDown
                if keyCode(escapeKey)
                    abort=true;
                end
            end
            if((GetSecs-starttime) >= wait_duration)
                %Snd('Play',MakeBeep(2000,0.15));
                wait_duration = 9999;
                go = 1;
            end
            if ((GetSecs-starttime) >= sampletime)
                tmp1 = GetSecs - starttime;
                for ii=1:repeats,
                    %read all mapped channel records
                    tmp = h.position;
                    data(sample,:) = [tmp1 toc(t0) go tmp];
                    sample = sample+1;
                end
                sampletime = sampletime + sample_interval;
            end
        end

        %Snd('Play',MakeBeep(600,0.1)/2);
        %Snd('Play',MakeBeep(300,0.04)/4);

        pn = length(data);
        if pn > 0,
            fprintf(' %d samples',pn);
            figure(figure_handle);
            set(figure_handle,'WindowStyle','docked');
            hold off;
            for sensor = 1:length(markers),
                plot3d(data(:,(sensor-1)*6+4:(sensor-1)*6+6),'.-','Color',col(markers(sensor)));
                hold on;
            end
            axis equal;
            hold off;
            save([pathname datafilename],'data','-ASCII','-TABS')
            fprintf(' saved to %s\n',datafilename);

            datafilename = increment_filenumber(datafilename);
            trial = trial + 1;
        else
            fprintf('WARNING *** PROBLEM WITH LIBERTY: collected %d samples\n',pn);
        end
    end
    h.stop;
    keep_going = getanswer('Run another session?',1);
end
% Disconnect
h.disconnect;
