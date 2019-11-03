function [cost, tau, xpredicted, ypredicted, zpredicted, Fx, Fy, Fz, Ftang] = minJerk(parameters, time, no_submovements, Gx, Gy, Gz)
    
    e = 0.0001;
    t0 = parameters(1,:);
    D = parameters(2,:);
    Dx = parameters(3,:);
    Dy = parameters(4,:);
    Dz = parameters(5,:);
    
    

    %Initialise tau
    tau = zeros(no_submovements,length(time));
    %Initialise xpredicted, ypredicted and zpredicted
    xpredicted = zeros(no_submovements,length(time));
    ypredicted = zeros(no_submovements,length(time));
    zpredicted = zeros(no_submovements,length(time));
    %Compute tau for all submovements and rands
    for submovem = 1:no_submovements
        t = time(time>=t0(submovem)&time<=(t0(submovem)+D(submovem)));
        sidx = find(abs(time-t(1))<=e);
        eidx = find(abs(time-t(end))<=e);
        tau_ind = (t - t0(submovem))./D(submovem);
        xpredicted_ind = (30*Dx(submovem)/D(submovem)).*(tau_ind.^4 - 2*tau_ind.^3 + tau_ind.^2);
        ypredicted_ind = (30*Dy(submovem)/D(submovem)).*(tau_ind.^4 - 2*tau_ind.^3 + tau_ind.^2);
        zpredicted_ind = (30*Dz(submovem)/D(submovem)).*(tau_ind.^4 - 2*tau_ind.^3 + tau_ind.^2);
        tau(submovem,sidx:eidx) = tau_ind;
        xpredicted(submovem,sidx:eidx) = xpredicted_ind;
        ypredicted(submovem,sidx:eidx) = ypredicted_ind;
        zpredicted(submovem,sidx:eidx) = zpredicted_ind;
    end
    
        Fx = sum(xpredicted,1);
        Fy = sum(ypredicted,1);
        Fz = sum(zpredicted,1);
        Ftang = sqrt(Fx.^2+Fy.^2+Fz.^2);
        Gtang = sqrt(Gx.^2+Gy.^2+Gz.^2);
        cost = sum((Fx - Gx).^2 + (Fy - Gy).^2 + (Fz - Gz).^2 + (Ftang - Gtang).^2);
end