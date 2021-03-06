%this uses the io32 driver found here
% http://apps.usd.edu/coglab/psyc770/IO32on64.html
% with instructions of how to set it up etc
%
%on PC-jaufr the parallel port is a SUNIX 1-port parallel card
% and I found its memory address with the device manage, under
% Multifunction adapters (The standard addresss is hex2dec('378');
%
%seems to take about 0.0084 ms for each output command
%
%Chris Miall 10 Oct 2019

ioObj = io32; %do not run this twice, without clearing it in between
status = io32(ioObj) %should be zero
io32address = hex2dec('1008');  %non-standard LPT1 output port address

%% test the output

data_out=255; %turn all 8 lines on
tic
io32(ioObj,io32address,data_out);
%this turns parallel data lines ON
pause(1)
io32(ioObj,io32address,0);
%this turns parallel data lines ON
toc-1

tic
data_out=bin2dec('01000000');
%this turns on data bits 3 and 7
io32(ioObj,io32address,data_out);
%this turns parallel data lines ON
pause(1)
io32(ioObj,io32address,0);
toc-1

tic
data_out=bin2dec('00100000');
%this turns on data bits 3 and 7
io32(ioObj,io32address,data_out);
%this turns parallel data lines ON
pause(1)
io32(ioObj,io32address,0);
toc-1

tic
data_out=bin2dec('01000010');
%this turns on data bits 3 and 7
io32(ioObj,io32address,data_out);
%this turns parallel data lines ON
pause(1)
io32(ioObj,io32address,0);
toc-1

tic
for ii=255:-1:0
    io32(ioObj,io32address,ii);
end
toc*1000/256

%% clear up
clear ioObj;
