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