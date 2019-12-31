data_out = '00000000';

for i = 1:100
    if i>5 && i<=30
        data_out = strcat(strrep(data_out(1),'0','1'),data_out(2:end));
        io32(ioObj,io32address,bin2dec(data_out));
        fprintf('\n port 7 on \n');
    elseif i>30
        data_out = strcat(strrep(data_out(1),'1','0'),data_out(2:end));
        io32(ioObj,io32address,bin2dec(data_out));
        fprintf('\n port 7 off \n');
    end
    
    if i>20 && i<=50
       data_out = strcat(data_out(1:4),strrep(data_out(5),'0','1'),data_out(6:end)); 
       io32(ioObj,io32address,bin2dec(data_out)); %start port 3
       fprintf('\n port 3 on \n');
    elseif i>50
        data_out = strcat(data_out(1:4),strrep(data_out(5),'1','0'),data_out(6:end)); 
        io32(ioObj,io32address,bin2dec(data_out)); %start port 3
        fprintf('\n port 3 off \n');
    end
    fprintf('\n %d \n',i);
end


    
%%

i = 1;
data_out = '00000000';
if i==1
    data_out = strcat(strrep(data_out(1),'0','1'),data_out(2:end));
    io32(ioObj,io32address,bin2dec(data_out));
    pause(5);
    data_out = strcat(data_out(1:4),strrep(data_out(5),'0','1'),data_out(6:end)); 
    io32(ioObj,io32address,bin2dec(data_out)); %start port 3
    pause(5);
    data_out = strcat(strrep(data_out(1),'1','0'),data_out(2:end));
    io32(ioObj,io32address,bin2dec(data_out));
    pause(1);
    data_out = strcat(data_out(1:4),strrep(data_out(5),'1','0'),data_out(6:end)); 
    io32(ioObj,io32address,bin2dec(data_out)); %start port 3
end

%% object 

%Parallel port object
ioObj = io32; %do not run this twice, without clearing it in between
status = io32(ioObj); %should be zero
io32address = hex2dec('1008');  %non-standard LPT1 output port address
data_out = '00000000';
data_out_cellarray = {@(data_out) strcat(strrep(data_out(1),'0','1'),data_out(2:end)), @(data_out) strcat(strrep(data_out(1),'1','0'),data_out(2:end));...
                      @(data_out) strcat(data_out(1:4),strrep(data_out(5),'0','1'),data_out(6:end)), @(data_out) strcat(data_out(1:4),strrep(data_out(5),'1','0'),data_out(6:end))};


%% test offline feedback
no_viapoints = 2;
success_history = [1 0 0 1; 1 0 1 0]; 
for p = 1:2
    if success_history(p,4) == 1
        start_vibration = tic;
        data_out = D(1,1);
        %data_out = strcat(strrep(data_out(1),'0','1'),data_out(2:end));
        io32(ioObj,io32address,bin2dec(data_out));
        pause(0.1);
        data_out = D(1,2);
        %data_out = strcat(strrep(data_out(1),'1','0'),data_out(2:end));
        io32(ioObj,io32address,bin2dec(data_out));
        pause(0.5)
    elseif success_history(p,4) == 0
        pause(1);
    end
end
   

%%
start = tic;
music = load('train');
played = false;
a = [];
while toc(start)<5
    a = [a; toc(start)];
    if played==false
        sound(music.y, music.Fs);
        played=true;
        b = toc(start);
    end
end

















































