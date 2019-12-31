%s1=ptb_init_serial_comms('Open','COM3',1200,7,'None',1,128);
%the USB-to-serial is identified as COM3 (but his might change when the
%Liberty system is plugged in)
set(s1,'DataTerminalReady','On');
%set(s1,'RequestToSend','On')

% % % tic; 
% % % set(s1,'DataTerminalReady','On');
% % % %this turns Pin4 of the USB-Serial-Parallel connection ON
% % % pause(1)
% % % set(s1,'DataTerminalReady','Off');
% % % %this turns Pin4 of the USB-Serial-Parallel connection OFF
% % % toc

tic
set(s1,'RequestToSend','Off')
%this turns Pin3 of the USB-Serial-Parallel connection ON
pause(1)
%set(s1,'RequestToSend','On')
%this turns Pin3 of the USB-Serial-Parallel connection OFF
toc
%ptb_init_serial_comms('Close');
% this isnt really necessary and you could leave the communications open
% until you finally switch matlab off


%seems to take about 4ms for each "set" command