% test_vibration_v1.m
%
% script to test vibration system with parallel port
%
% Chris Miall June 25 2019

%% set up parallel port
%unlike ptb_init_parallel_port, ControlBits are inputs for VoiceKey
try
    % Create a DIO object using the specified port
    dio=digitalio('parallel','LPT1');  % Create Digital I/O object

    % Add lines for the Data Interface
    DataOUTBits=addline(dio,0:7,0,'out');	% Data Port: Pin 2-9
    putvalue(DataOUTBits,0);

catch
    rethrow(lasterror)
    %return keep going to debug
end

%% run the test
disp('Experiment to test vibration system')
while getanswer('Test it now?',1)
    fprintf('\nTURN ON VIBRATION ...');
    putvalue(DataOUTBits,255); %all 8 lines on
    pause(1)
    putvalue(DataOUTBits,0); %all lines off
    fprintf('TURN OFF VIBRATION \n');
end
%% reset
putvalue(DataOUTBits,0); %outputs off
daqreset;

