DIO = digitalio('mcc',0); %ID board #0 needed here in case the MCC 1048 48-channel IO is also installed
%add 3 output lines eg 2x vibration channels, one bleeper channel

lines=addline(DIO,0:2,'out');
%turn outputs off
putvalue(DIO,0);


%data = 0;

%while data,
    %data = getnumber('Line numbers as binary or zero to quit)',1);
    %set up a binary vector with 3 values to control the three outputs
    %0= all off
    %1=channel 1 only
    %2=cahnnel 2 only
    %3=channel 3 = 1 & 2
    %binvec = dec2binvec(data,3)
    %putvalue(DIO,binvec);
%end

% n = 0;
% for i=1:100
%     n = n+1;
%     if n>5 && n<10
%         putvalue(DIO,[1 0 0]);
%         fprintf('\n start');
%     else
%         putvalue(DIO,[0 0 0]);
%         fprintf('\n stop');
%     end
% end


putvalue(DIO,[1 0 0]);
pause(0.15);
putvalue(DIO,[0 0 0]);