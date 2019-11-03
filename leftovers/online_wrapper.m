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
hFig = figure();
hold on;
set(hFig,'WindowStyle','docked');
hTxt=text(40,-40,-40,'waiting for data');
hDat=plot3(NaN,NaN,NaN,'+','MarkerSize',14);
set(gca,'xlim',[0 50],'ylim',[-50 50],'zlim',[-50 50],'Zdir','reverse');
xlabel('X');
ylabel('Y');
zlabel('Z');

%!!! PREALLOCATION
step=50; %step 50 
vx = zeros(1,length(markers));
vy = zeros(1,length(markers));
vz = zeros(1,length(markers));
data_test = [];
Vmag_total=[];
Vmag=zeros(1,length(markers));
tocvt=[];

%Filtering coefficients
cutoff = 15;
wn = cutoff/(rate/2);
[b,a] = butter(4,wn);

%axis equal; keep t0=tic close to toc(t0)
view(120,30);
hold off;
h.start;
data=[];
t0=tic;
sample=0;
error_sound=0;

while ishandle(hFig), %until figure is closed
    tmp = h.position;
    timestamp = toc(t0);
    %fprintf('x y x %d %d %d\n', tmp([1,2,3]));
    %if ~isempty(tmp)
        if timestamp<3 %&& not at target yet 
        elseif timestamp>3 && timestamp<5 %&& not at target yet
            %if not at target think how you do this so that you do not
            %check if you reached the target every timestamp following time
            %overshoot
%             if error_sound==0
%                 fprintf('\n ERROR');
%                 error_sound=1;
%             end
            %else
            %end
             tmp = zeros(1,6*length(markers));
        elseif timestamp>5
             error_sound=0;
             t0 = tic; %move this up
            %h.wait;
       end
 
        data = [data; timestamp tmp];
        
        if size(data,1)>= step
            data_tofilter = data((size(data,1)-(step-2)):end,2:end);
            data_filtered = filter(b,a,data_tofilter); 
            %data(end,2:end) = data_filtered(end,:); uncomm delete line
            %below
            data_test = [data_test; data_filtered(end,:)];
        %!!!!!velocity - first, for one marker only! filter all data     
       
            for i=1:length(markers)
                vx(:,i) = mean(gradient(data((size(data,1)-(step-1)):end,(6*i-4)),(1/rate)));
                vy(:,i) = mean(gradient(data((size(data,1)-(step-1)):end,(6*i-2)),(1/rate)));
                vz(:,i) = mean(gradient(data((size(data,1)-(step-1)):end,(6*i-3)),(1/rate)));
            end

            V = [vx vy vz];

            for i = 1:length(markers)
                    Vmag(i) = norm(V(i:size(markers,2):end));
            end
        end
            
            Vmag_total = [Vmag_total; Vmag];
%             tocv = toc(tv);
%             tocvt = [tocvt tocv];
        
        set(hTxt,'String',sprintf('Data found at time %d',timestamp)); %replace with toc(t0)
        tmp1=reshape(tmp,6,sens)';
        set(hDat,'xdata',tmp1(:,1),'ydata',tmp1(:,2),'zdata',tmp1(:,3));
        drawnow();
end
    sample=sample+sample_interval;
    %while timestamp<=sample %replace with toc(t0)
    %end
%end

h.stop;
% Disconnect
h.disconnect;
%fprintf('\n Approx sample rate was %f per second\n',1/mean(diff(data(:,1))));