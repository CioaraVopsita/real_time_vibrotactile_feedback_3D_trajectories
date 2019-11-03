%bounds for amplitude: if amplitude = displacement; max displacement in
%0.0042s = 7cm => 1680 cm in 1s => ~ 17m in a sec

lb = [0 0.167 -15 -15 -15];
up = [t_total-0.167 1 15 15 15];


%reconstructed velocity profile => x_deriv_predicted(t) for all t between
%t0 and tfinal

for no_submovements = 1:10
    %Random parameters
    Dx = -15+30*rand(no_submovements,10);Dy = -15+30*rand(no_submovements,10);Dz = -15+30*rand(no_submovements,10);
    t0 = (time(length(time))-0.167)*rand(no_submovements,10);D=0.167+0.833*rand(no_submovements,10);
    %Initialise tau
    tau = zeros(no_submovements,10,length(time));
    %Initialise xpredicted, ypredicted and zpredicted
    xpredicted = zeros(no_submovements,10,length(time));
    ypredicted = zeros(no_submovements,10,length(time));
    zpredicted = zeros(no_submovements,10,length(time));
    %Compute tau for all submovements and rands
    for submovement = 1:no_submovements
        for rands = 1:10
            t = time(time>=t0(submovement,rands)&time<=(t0(submovement,rands)+D(submovement,rands)));
            sidx = find(time==t(1));
            eidx = find(time==t(end));
            tau_ind = (t - t0(submovement,rands))./D(submovement,rands);
            xpredicted_ind = (30*Dx(submovement,rands)/D(submovement,rands)).*(tau_ind.^4 - 2*tau_ind.^3 + tau_ind.^2);
            ypredicted_ind = (30*Dy(submovement,rands)/D(submovement,rands)).*(tau_ind.^4 - 2*tau_ind.^3 + tau_ind.^2);
            zpredicted_ind = (30*Dz(submovement,rands)/D(submovement,rands)).*(tau_ind.^4 - 2*tau_ind.^3 + tau_ind.^2);
            tau(submovement,rands,sidx:eidx) = tau_ind;
            xpredicted(submovement,rands,sidx:eidx) = xpredicted_ind;
            ypredicted(submovement,rands,sidx:eidx) = ypredicted_ind;
            zpredicted(submovement,rands,sidx:eidx) = zpredicted_ind;   
        end
    end
end
cost = @(parameters) 

%parameters for each submovement