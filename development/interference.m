DIO = digitalio('mcc',0); %ID board #0 needed here in case the MCC 1048 48-channel IO is also installed
%add 3 output lines eg 2x vibration channels, one bleeper channel

lines=addline(DIO,0:2,'out');
%turn outputs off
putvalue(DIO,0);

DIO.TimerPeriod = 0.1;

% queueOutputData(s,linspace(-1,1,1000)');
% startForeground(s)

t0 = tic;
sample = 0;
sample_interval = 1/240;
n = 1;
data = [];
vibration = false;

while toc(t0)<5
    
    if vibration
        if toc(start_vibration)>1
            putvalue(DIO,[0 0 0]);
            vibration=false;
        end
    end
    
    n = n+1;
    data = [data; toc(t0) n];
    sample=sample+sample_interval;
    
    if n == 242
        start_vibration = tic;
        putvalue(DIO,[1 0 0]);
        vibration = true;
    end
    
    while toc(t0)<=sample %replace with toc(t0) falls behind => the 10 identical datapoints after pulse
    end
end