%% Process concurrent feedback data

no_blocks = 4;
no_trials_inblock = 50;
no_participants = 7;

jerk_matrix_c = zeros(no_blocks,no_trials_inblock,no_participants);
hits_matrix_c = zeros(3,2,no_blocks,no_participants);

for ii = 1:no_participants
    for jj = 1:no_blocks
        load([num2str(ii) 'c_' num2str(jj) '.mat']);
        %Compute square jerk (from Vtang)
        for mm = 1:no_trials_inblock
            data_trial = data{mm};
            time = data_trial(:,1)';
            velocity = Vtang{mm};
            padding = length(time)-length(velocity);
            velocity = [zeros(1,padding), velocity];
            jerk_vector = gradient(gradient(velocity, 1/rate), 1/rate);
            trajectory_jerk = sum(jerk_vector.^2);
            
            %Array that contains the squared jerk values for each
            %trajectory, block and participant
            jerk_matrix_c(jj,mm,ii) = trajectory_jerk;
        end
        
        vp_hits = sum(success_history(:,:,1,1),2);
        vel_hits = sum(success_history(:,:,1,2),2);
        hits = [vp_hits vel_hits];
        
        %Array that contains the number of successful reaches of the
        %viapoint/smoothness for each participant and block 
        hits_matrix_c(:,:,jj,ii)=hits;
    end
end

%% Process offline feedback data

no_blocks = 4;
no_trials_inblock = 50;
no_participants = 7;

jerk_matrix_o = zeros(no_blocks,no_trials_inblock,no_participants);
hits_matrix_o = zeros(3,2,no_blocks,no_participants);

for ii = 1:no_participants
    for jj = 1:no_blocks
        load([num2str(ii) 'o_' num2str(jj) '.mat']);
        %Compute square jerk (from Vtang)
        for mm = 1:no_trials_inblock
            data_trial = data{mm};
            time = data_trial(:,1)';
            velocity = Vtang{mm};
            padding = length(time)-length(velocity);
            velocity = [zeros(1,padding), velocity];
            jerk_vector = gradient(gradient(velocity, 1/rate), 1/rate);
            trajectory_jerk = sum(jerk_vector.^2);
            
            %Array that contains the squared jerk values for each
            %trajectory, block and participant
            jerk_matrix_o(jj,mm,ii) = trajectory_jerk;
        end
        
        vp_hits = sum(success_history(:,:,1,1),2);
        vel_hits = sum(success_history(:,:,1,2),2);
        hits = [vp_hits vel_hits];
        
        %Array that contains the number of successful reaches of the
        %viapoint/smoothness for each participant, block and trajectory
        hits_matrix_o(:,:,jj,ii)=hits;
    end
end

%% Statistical analysis

%Reshape matrices for analysis

jerk_matrix_c = reshape(jerk_matrix_c,4,350)'; %50trials_per_block*no_participants x no_blocks
jerk_matrix_o = reshape(jerk_matrix_o,4,350)';
jerk_matrix = [jerk_matrix_c; jerk_matrix_o];

hits_matrix_c_position = permute(squeeze(hits_matrix_c(1:2,1,:,:)), [3,2,1]);
hits_matrix_o_position = permute(squeeze(hits_matrix_o(1:2,1,:,:)), [3,2,1]);
hits_matrix_position = [hits_matrix_c_position; hits_matrix_o_position]; %no_participants x blocks x viapoint

hits_matrix_c_velocity = permute(squeeze(hits_matrix_c(:,2,:,:)), [3,2,1]);
hits_matrix_o_velocity = permute(squeeze(hits_matrix_o(:,2,:,:)), [3,2,1]);
hits_matrix_velocity = [hits_matrix_c_velocity; hits_matrix_o_velocity]; %no_participants x blocks x velocity_segment

%% Jerk ANOVAs and t-test

% Jerk 2x4 ANOVA
figure(1)
[~,~,stats_jerk_all] = anova2(jerk_matrix,350);
figure(2)
c_jerk_all = multcompare(stats_jerk_all);

% Jerk 1x4 ANOVA for concurrent and offline feedback respectively
figure(3)
[~,~,stats_jerk_c] = anova2(jerk_matrix_c,350);
figure(4)
c_jerk_c = multcompare(stats_jerk_c);

figure(5)
[~,~,stats_jerk_o] = anova2(jerk_matrix_o,350);
figure(6)
c_jerk_o = multcompare(stats_jerk_o);

% Jerk t-tests - compare concurrent and offline feedback for first and
% last block respectively
block1 = [jerk_matrix_c(:,1) jerk_matrix_o(:,1)];
[h_block1, p_block1] = ttest2(block1(:,1), block1(:,2));

block4 = [jerk_matrix_c(:,4) jerk_matrix_o(:,4)];
[h_block4, p_block4] = ttest2(block4(:,1), block4(:,2));

%% Viapoint hits
% Hits position 2x4x2 ANOVA
position_hits_vector = [];
condition_position = [];
block_position = [];
segment_position = [];

for iii=1:14
    for jjj=1:4
        for kkk=1:2
            position_hits_vector = [position_hits_vector hits_matrix_position(iii,jjj,kkk)];
            if iii<=7
                cond = 1;
            elseif iii>7
                cond = 2;
            end
            condition_position = [condition_position cond];
            block_position = [block_position jjj];
            segment_position = [segment_position kkk];
        end
    end
end

[~,~,stats_position_all] = anovan(position_hits_vector,{condition_position block_position segment_position},'model','interaction',...
    'varnames',{'condition','block','viapoint'});

% Plot interactions
cc = mean(hits_matrix_c_position, [1,2]);
avgHits_vp1_c = cc(:,:,1);
avgHits_vp2_c = cc(:,:,2);
oo = mean(hits_matrix_o_position, [1,2]);
avgHits_vp1_o = oo(:,:,1);
avgHits_vp2_o = oo(:,:,2);

%Not an indication of correct trajectory
plot([1,5],[avgHits_vp1_c, avgHits_vp1_o], '*-');
hold on
plot([1,5],[avgHits_vp2_c, avgHits_vp2_o], 'o-');

%% Velocity hits

velocity_hits_vector = [];
condition_velocity = [];
block_velocity = [];
segment_velocity = [];

% Hits velocity 2x4x3 ANOVA
for iii=1:14
    for jjj=1:4
        for kkk=1:3
            velocity_hits_vector = [velocity_hits_vector hits_matrix_velocity(iii,jjj,kkk)];
            if iii<=7
                condv = 1;
            elseif iii>7
                condv = 2;
            end
            condition_velocity = [condition_velocity condv];
            block_velocity = [block_velocity jjj];
            segment_velocity = [segment_velocity kkk];
        end
    end
end

[~,~,stats_velocity_all] = anovan(velocity_hits_vector,{condition_velocity block_velocity segment_velocity},'model','interaction',...
    'varnames',{'condition','block','submovement'});










