%% gradient with step, SI for position
%Offline!
%sampling: 0.0042
%every fifth => 0.021
%instead of hard-coding a number, 10^(-2)/((1/rate)*step)
rate = 240;
step = 5;
%!preallocate

markers = [1];

for i=1:size(markers,2)
    vx(:,i) = gradient(mov_002(1:end,(6*i-2)),(1/rate))*10^(-2);
    vy(:,i) = gradient(mov_002(1:end,6*i),(1/rate))*10^(-2);
    vz(:,i) = gradient(mov_002(1:end,(6*i-1)),(1/rate))*10^(-2);
end
    V = [vx vy vz];
    
for i = 1:size(markers,2)
    for j=1:size(V,1)
        Vmag(j,i) = norm(V(j,i:size(markers,2):end));
    end
end
plot(Vmag)
%% diff

%Offline!
%sampling: 0.0042
%every fifth => 0.021
%instead of hard-coding a number, 10^(-2)/((1/rate)*step)
rate = 240;
step = 1;
%!preallocate

markers = [1];

for i=1:size(markers,2)
    vx(:,i) = diff(junk_001(1:5:end,(6*i-2)))*10^(-2)/((1/rate)*step);
    vy(:,i) = diff(junk_001(1:5:end,6*i))*10^(-2)/((1/rate)*step);
    vz(:,i) = diff(junk_001(1:5:end,(6*i-1)))*10^(-2)/((1/rate)*step);
end
    V = [vx vy vz];
    
for i = 1:size(markers,2)
    for j=1:size(V,1)
        Vmag(j,i) = norm(V(j,i:size(markers,2):end));
    end
end
plot(Vmag)
%%
%online test
rg_total=[];
for i=1:length(r)
    if mod(i,3)==0
        rg = gradient(r((i-2):i));
        rg_total = [rg_total rg];
    end
end

%%
    vx = gradient(sq_003(2:end,4),(1/240))*10^(-2);
    vy = gradient(sq_003(2:end,5),(1/240))*10^(-2);
    vz = gradient(sq_003(2:end,6),(1/240))*10^(-2);
    
    V = [vx vy vz];

 for i=1:size(V,1)
        Vmag(i) = norm(V(i,:));
 end
 plot(Vmag)

