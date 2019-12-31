function [cost, gradient, hessian, Ftang, Gtang, Ftang2, xpredicted, ypredicted, zpredicted] = submovement(parameters, time, no_submovements, Gx, Gy, Gz)
    
    
    e = 0.0001;
    
    D = parameters(1,:);
    t0 = parameters(2,:);
    Ax = parameters(3,:);
    Ay = parameters(4,:);
    Az = parameters(5,:);
    
    

    %Initialise tau
    tau = zeros(no_submovements,length(time));
    %Initialise xpredicted, ypredicted and zpredicted
    xpredicted = zeros(no_submovements,length(time));
    ypredicted = zeros(no_submovements,length(time));
    zpredicted = zeros(no_submovements,length(time));
    vtangpredicted = zeros(no_submovements, length(time));
    
    Dx = zeros(5, length(time)); Dx_total = [];
    Dy = zeros(5, length(time)); Dy_total = [];
    Dz = zeros(5, length(time)); Dz_total = [];
    Dtang = zeros(5, length(time)); Dtang_total = [];
    
    Hx = zeros(5,5,length(time)); Hx_total = [];
    Hy = zeros(5,5,length(time)); Hy_total = [];
    Hz = zeros(5,5,length(time)); Hz_total = [];
    Htang = zeros(5,5,length(time)); Htang_total = [];
    
    %Initialise gradient and hessian matrices
    gradient = zeros(5*no_submovements,length(time));
    hessian = zeros(5*no_submovements,5,length(time));
    
    %Compute tau for all submovements and rands
    for submovem = 1:no_submovements
        t = time(time>=t0(submovem)&time<=(t0(submovem)+D(submovem)));
        sidx = find(abs(time-t(1))<=e);
        eidx = find(abs(time-t(end))<=e);
        [velx_predicted, vely_predicted, velz_predicted, vel_tang, firstx, firsty, firstz, firsttang, secondx, secondy, secondz, secondtang] = minjerk(t, t0(submovem), Ax(submovem), Ay(submovem), Az(submovem),D(submovem));
        %tau_ind = (t - t0(submovem))./D(submovem);
        %xpredicted_ind = (30*Dx(submovem)/D(submovem)).*(tau_ind.^4 - 2*tau_ind.^3 + tau_ind.^2);
        %ypredicted_ind = (30*Dy(submovem)/D(submovem)).*(tau_ind.^4 - 2*tau_ind.^3 + tau_ind.^2);
        %zpredicted_ind = (30*Dz(submovem)/D(submovem)).*(tau_ind.^4 - 2*tau_ind.^3 + tau_ind.^2);
        %tau(submovem,sidx:eidx) = tau_ind;
        xpredicted(submovem,sidx:eidx) = velx_predicted;
        ypredicted(submovem,sidx:eidx) = vely_predicted;
        zpredicted(submovem,sidx:eidx) = velz_predicted;
        vtangpredicted(submovem,sidx:eidx) = vel_tang;
        
        %First order partial derviatives placed in zeros matrices and then
        % across submovements
        Dx(:,sidx:eidx) = firstx;
        Dx_total = [Dx_total; Dx];
        Dy(:,sidx:eidx) = firsty;
        Dy_total = [Dy_total; Dy];
        Dz(:,sidx:eidx) = firstz;
        Dz_total = [Dz_total; Dz];
        Dtang(:,sidx:eidx) = firsttang;
        Dtang_total = [Dtang_total; Dtang];
        
        
        
        %Hessian
        Hx(:,:,sidx:eidx) = secondx;
        Hx_total = [Hx_total; Hx];
        Hy(:,:,sidx:eidx) = secondy;
        Hy_total = [Hy_total; Hy];
        Hz(:,:,sidx:eidx) = secondz;
        Hz_total = [Hz_total; Hz];
        Htang(:,:,sidx:eidx) = secondtang;
        Htang_total = [Htang_total; Htang];
        
    end
    
%   Rearranging first order partial derivatives    
%         Dxd(1:2,:) = Dx_temp([1 6],:);
%         Dxt(1:2,:) = Dx_temp([2 7],:);
%         DxAx(1:2,:)=Dx_temp([3 8],:);
%         DxAy(1:2,:)=Dx_temp([4 9],:);
%         DxAz(1:2,:)=Dx_temp([5 10],:);
%         Dx_total = [Dxd; Dxt; DxAx; DxAy; DxAz];
%         
%         Dyd(1:2,:) = Dy_temp([1 6],:);
%         Dyt(1:2,:) = Dy_temp([2 7],:);
%         DyAx(1:2,:)=Dy_temp([3 8],:);
%         DyAy(1:2,:)=Dy_temp([4 9],:);
%         DyAz(1:2,:)=Dy_temp([5 10],:);
%         Dy_total = [Dyd; Dyt; DyAx; DyAy; DyAz];
%         
%         
%         Dzd(1:2,:) = Dz_temp([1 6],:);
%         Dzt(1:2,:) = Dz_temp([2 7],:);
%         DzAx(1:2,:)=Dz_temp([3 8],:);
%         DzAy(1:2,:)=Dz_temp([4 9],:);
%         DzAz(1:2,:)=Dz_temp([5 10],:);
%         Dz_total = [Dzd; Dzt; DzAx; DzAy; DzAz];
%         
%         Dtangd(1:2,:) = Dtang_temp([1 6],:);
%         Dtangt(1:2,:) = Dtang_temp([2 7],:);
%         DtangAx(1:2,:)=Dtang_temp([3 8],:);
%         DtangAy(1:2,:)=Dtang_temp([4 9],:);
%         DtangAz(1:2,:)=Dtang_temp([5 10],:);
%         Dtang_total = [Dtangd; Dtangt; DtangAx; DtangAy; DtangAz];
    
        Fx = sum(xpredicted,1);
        Fy = sum(ypredicted,1);
        Fz = sum(zpredicted,1);
        Ftang = sum(vtangpredicted,1);
        Ftang2 = sqrt(Fx.^2+Fy.^2+Fz.^2);
        Gtang = sqrt(Gx.^2+Gy.^2+Gz.^2);
        %Gtang(Gtang==0) = rep;
        
        %!!!!!!!!!squaring values < 1 velocity uses m/s...cm/s?
        %cost = sum(((Fx - Gx).^2 + (Fy - Gy).^2 + (Fz - Gz).^2 + (Ftang - Gtang).^2));
        
        if nargout>1
            for k=1:size(gradient,1)
                gradient(k,:) = 2*((Fx-Gx).*Dx_total(k,:)+ ...
                                (Fy-Gy).*Dy_total(k,:)+ ...
                                (Fz-Gz).*Dz_total(k,:)+ ...
                                (Ftang-Gtang).*Dtang_total(k,:));
            end
            gradient = sum(gradient,2);
            if nargout>2
               % hessian
               for i=1:size(hessian,1)
                   for j=1:size(hessian,2)
                       hessian(i,j,:) = 2*(Dx_total(i,:).*Dx_total(j,:) + ((Fx - Gx).* squeeze(Hx_total(i,j,:))') + ...
                           Dy_total(i,:).*Dy_total(j,:) + ((Fy - Gy).* squeeze(Hy_total(i,j,:))') + ...
                           Dz_total(i,:).*Dz_total(j,:) + ((Fz - Gz).* squeeze(Hz_total(i,j,:))')+...
                           Dtang_total(i,:).*Dtang_total(j,:) + ((Ftang - Gtang).* squeeze(Htang_total(i,j,:))'));
                   end
               end
               hessian = sum(hessian,3);
            end
        end
        
     cost = sum(((Fx - Gx).^2 + (Fy - Gy).^2 + (Fz - Gz).^2 + (Ftang - Gtang).^2));
end
            
            
            
            