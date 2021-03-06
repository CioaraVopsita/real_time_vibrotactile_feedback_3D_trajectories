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
[rr,ss,stats_jerk_all] = anova2(jerk_matrix,350);
figure(2)
c_jerk_all = multcompare(stats_jerk_all);

% Jerk 1x4 ANOVA for concurrent and offline feedback respectively
figure(3)
[~,sr,stats_jerk_c] = anova1(jerk_matrix_c);
y1 = stats_jerk_c.means;
figure(4)
c_jerk_c = multcompare(stats_jerk_c);

figure(5)
[~,rs,stats_jerk_o] = anova1(jerk_matrix_o);
y2 = stats_jerk_o.means;
figure(6)
c_jerk_o = multcompare(stats_jerk_o);

X = categorical({'Concurrent','Terminal'});
X = reordercats(X,{'Concurrent','Terminal'});
Y = [y1;y2];
bar(X,Y)

% Jerk t-tests - compare concurrent and offline feedback for first and
% last block respectively
block1 = [jerk_matrix_c(:,1) jerk_matrix_o(:,1)];
[h_block1, p_block1,~,statss] = ttest2(block1(:,1), block1(:,2));

block4 = [jerk_matrix_c(:,4) jerk_matrix_o(:,4)];
[h_block4, p_block4,~,stats] = ttest2(block4(:,1), block4(:,2));

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
X = categorical({'Concurrent','Terminal'});
plot(X,[avgHits_vp1_c, avgHits_vp1_o], '*-');
hold on
plot(X,[avgHits_vp2_c, avgHits_vp2_o], 'o-');

%One-way ANOVAs to test for simple effects of viapoints in each condition
vect_simple_eff_c_position = reshape(hits_matrix_c_position, [28,2]);
[~,vp1anova] = anova1(vect_simple_eff_c_position);
vect_simple_eff_o_position = reshape(hits_matrix_o_position, [28,2]);
[~,vp2anova] = anova1(vect_simple_eff_o_position);

%One-way ANOVAs to test for simple effects of condition on each viapoint
condOnViapoint1 = [vect_simple_eff_c_position(:,1) vect_simple_eff_o_position(:,1)];
[~,condvp1] = anova1(condOnViapoint1); %SIGNIFICANT
condOnViapoint2 = [vect_simple_eff_c_position(:,2) vect_simple_eff_o_position(:,2)];
[~,condvp2] = anova1(condOnViapoint2);
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

% Plot interactions
ccv = mean(hits_matrix_c_velocity, [1,2]);
avgHits_sub1_c = ccv(:,:,1);
avgHits_sub2_c = ccv(:,:,2);
avgHits_sub3_c = ccv(:,:,3);
oov = mean(hits_matrix_o_velocity, [1,2]);
avgHits_sub1_o = oov(:,:,1);
avgHits_sub2_o = oov(:,:,2);
avgHits_sub3_o = oov(:,:,3);

%Not an indication of correct trajectory
plot(X,[avgHits_sub1_c, avgHits_sub1_o], '*-');
hold on
plot(X,[avgHits_sub2_c, avgHits_sub2_o], 'o-');
plot(X,[avgHits_sub3_c, avgHits_sub3_o], '.-');

%One-way ANOVAs to test for simple effects of submovements in each condition
vect_simple_eff_c_velocity = reshape(hits_matrix_c_velocity, [28,3]);
[~,sub1anova,tb1] = anova1(vect_simple_eff_c_velocity);
vect_simple_eff_o_velocity = reshape(hits_matrix_o_velocity, [28,3]);
[~,sub2anova,tb2] = anova1(vect_simple_eff_o_velocity);

tttc = multcompare(tb1); %sub1 signfic different from the others
ttto = multcompare(tb2); %all significantly different
%for both conditions hits steadily decreasing

%One-way ANOVAs to test for simple effects of condition on each submovement
condOnSub1 = [vect_simple_eff_c_velocity(:,1) vect_simple_eff_o_velocity(:,1)];
[~,condsub1] = anova1(condOnSub1);  %right velocity for the first submovement is more reliably reached in the offline condition; consistent with the first vp being more frequently reached in the conccurrent condition.
condOnSub2 = [vect_simple_eff_c_velocity(:,2) vect_simple_eff_o_velocity(:,2)];
[~,condsub2] = anova1(condOnSub2);
condOnSub3 = [vect_simple_eff_c_velocity(:,3) vect_simple_eff_o_velocity(:,3)];
[~,condsub3] = anova1(condOnSub3);

%% More plots

X = categorical({'Concurrent','Terminal'});
X = reordercats(X,{'Concurrent','Terminal'});
Y = [1.93506E+12 1.01888E+12];
bar(X,Y,0.5)

X = categorical({'Block1','Block2','Block3','Block4'});
X = reordercats(X,{'Block1','Block2','Block3','Block4'});
Y = [9.09053E+11 6.14165E+11 1.71093E+12 2.67372E+12];
bar(X,Y,0.7)

X = categorical({'Concurrent','Terminal'});
X = reordercats(X,{'Concurrent','Terminal'});
Y = [7.892857143 14.39285714; 1.071428571 18];
bar(X,Y);

X = categorical({'Concurrent','Terminal'});
X = reordercats(X,{'Concurrent','Terminal'});
Y = [21.35714286 11.85714286 9.785714286; 26.32142857 11.82142857 6.75];
bar(X,Y);








