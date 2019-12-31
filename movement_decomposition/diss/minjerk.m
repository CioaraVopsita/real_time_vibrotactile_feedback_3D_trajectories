function [velx_predicted, vely_predicted, velz_predicted, vel_tang, firstx, firsty, firstz, firsttang, secondx, secondy, secondz, secondtang] = minjerk(time, t0, Ax, Ay, Az,d)
   
    
    %Normalised time
    tau = (time-t0)./d;
    
    %Min-jerk velocity profiles
    velx_predicted = (30*Ax/d).*(tau.^4 - 2*tau.^3 + tau.^2);
    vely_predicted = (30*Ay/d).*(tau.^4 - 2*tau.^3 + tau.^2);
    velz_predicted = (30*Az/d).*(tau.^4 - 2*tau.^3 + tau.^2);
    vel_tang = 30/d*sqrt(Ax^2+Ay^2+Az^2).*(tau.^4-2*tau.^3+tau.^2);
    
    if nargout>3
        %First oder partial derivatives: d, t0, Ax, Ay, Az
        firstx(1,:) = (-30*Ax/d^2).*(3*tau.^2-8*tau.^3+5*tau.^4); %dFx/dd
        firstx(2,:) = (-60*Ax/d^2).*(tau-3*tau.^2+2*tau.^3); %dFx/dt0
        firstx(3,:) = 30/d.*(tau.^2-2*tau.^3+tau.^4); %dFx/Ax etc
        firstx(4,:) = zeros(size(time));
        firstx(5,:) = zeros(size(time));
        
        firsty(1,:) = (-30*Ay/d^2).*(3*tau.^2-8*tau.^3+5*tau.^4); %dFy/dd
        firsty(2,:) = (-60*Ay/d^2).*(tau-3*tau.^2+2*tau.^3); %dFy/dt0 etc
        firsty(3,:) = zeros(size(time));
        firsty(4,:) = 30/d.*(tau.^2-2*tau.^3+tau.^4);
        firsty(5,:) = zeros(size(time));
        
        firstz(1,:) = (-30*Az/d^2).*(3*tau.^2-8*tau.^3+5*tau.^4);
        firstz(2,:) = (-60*Az/d^2).*(tau-3*tau.^2+2*tau.^3);
        firstz(3,:) = zeros(size(time));
        firstz(4,:) = zeros(size(time));
        firstz(5,:) = 30/d.*(tau.^2-2*tau.^3+tau.^4);
        
        firsttang(1,:) = -30/d^2*sqrt(Ax^2+Ay^2+Az^2).*(5*tau.^4-8*tau.^3+3*tau.^2); %correct
        firsttang(2,:) = -60/d^2*sqrt(Ax^2+Ay^2+Az^2).*(2*tau.^3-3*tau.^2+tau);
        firsttang(3,:) = 30/d*Ax*(1/sqrt(Ax^2+Ay^2+Az^2)).*(tau.^4-2*tau.^3+tau.^2);
        firsttang(4,:) = 30/d*Ay*(1/sqrt(Ax^2+Ay^2+Az^2)).*(tau.^4-2*tau.^3+tau.^2);
        firsttang(5,:) = 30/d*Az*(1/sqrt(Ax^2+Ay^2+Az^2)).*(tau.^4-2*tau.^3+tau.^2); 
  
        if nargout>6
            %Hessian - second order partial derivatives: d, t0, Ax, Ay, Az
            secondx(1,1,:) = (60*Ax/d^3).*(6*tau.^2-20*tau.^3+15*tau.^4);
            secondx(1,2,:) = (60*Ax/d^3).*(3*tau-12*tau.^2+10*tau.^3);
            secondx(1,3,:) = (-30/d^2).*(3*tau.^2-8*tau.^3+5*tau.^4);
            secondx(1,4,:) = zeros(size(time));
            secondx(1,5,:) = zeros(size(time));
            secondx(2,1,:) = (60*Ax/d^3).*(3*tau-12*tau.^2+10*tau.^3);
            secondx(2,2,:) = (60*Ax/d^3).*(1-6*tau+6*tau.^2);
            secondx(2,3,:) = (-60/d^2).*(tau-3*tau.^2+2*tau.^3);
            secondx(2,4,:) = zeros(size(time));
            secondx(2,5,:) = zeros(size(time));
            secondx(3,1,:) = (-30/d^2).*(3*tau.^2-8*tau.^3+5*tau.^4);
            secondx(3,2,:) = (30/d^2).*(-2*tau+6*tau.^2-4*tau.^3);
            secondx(3,3,:) = zeros(size(time));
            secondx(3,4,:) = zeros(size(time));
            secondx(3,5,:) = zeros(size(time));
            secondx(4,1,:) = zeros(size(time));
            secondx(4,2,:) = zeros(size(time));
            secondx(4,3,:) = zeros(size(time));
            secondx(4,4,:) = zeros(size(time));
            secondx(4,5,:) = zeros(size(time));
            secondx(5,1,:) = zeros(size(time));
            secondx(5,2,:) = zeros(size(time));
            secondx(5,3,:) = zeros(size(time));
            secondx(5,4,:) = zeros(size(time));
            secondx(3,5,:) = zeros(size(time));
            
            %rest for Hy and Hz: d, t0, Ax, Ay, Az
            
            secondy(1,1,:) = (60*Ay/d^3).*(6*tau.^2-20*tau.^3+15*tau.^4);
            secondy(1,2,:) = (60*Ay/d^3).*(3*tau-12*tau.^2+10*tau.^3);
            secondy(1,3,:) = zeros(size(time));
            secondy(1,4,:) = (-30/d^2).*(3*tau.^2-8*tau.^3+5*tau.^4);
            secondy(1,5,:) = zeros(size(time));
            secondy(2,1,:) = (60*Ay/d^3).*(3*tau-12*tau.^2+10*tau.^3);
            secondy(2,2,:) = (60*Ay/d^3).*(1-6*tau+6*tau.^2);
            secondy(2,3,:) = zeros(size(time));
            secondy(2,4,:) = (-60/d^2).*(tau-3*tau.^2+2*tau.^3);
            secondy(2,5,:) = zeros(size(time));
            secondy(3,1,:) = (-30/d^2).*(3*tau.^2-8*tau.^3+5*tau.^4);
            secondy(3,2,:) = (30/d^2).*(-2*tau+6*tau.^2-4*tau.^3);
            secondy(3,3,:) = zeros(size(time));
            secondy(3,4,:) = zeros(size(time));
            secondy(3,5,:) = zeros(size(time));
            secondy(4,1,:) = zeros(size(time));
            secondy(4,2,:) = zeros(size(time));
            secondy(4,3,:) = zeros(size(time));
            secondy(4,4,:) = zeros(size(time));
            secondy(4,5,:) = zeros(size(time));
            secondy(5,1,:) = zeros(size(time));
            secondy(5,2,:) = zeros(size(time));
            secondy(5,3,:) = zeros(size(time));
            secondy(5,4,:) = zeros(size(time));
            secondy(5,5,:) = zeros(size(time));
            
            secondz(1,1,:) = (60*Az/d^3).*(6*tau.^2-20*tau.^3+15*tau.^4);
            secondz(1,2,:) = (60*Az/d^3).*(3*tau-12*tau.^2+10*tau.^3);
            secondz(1,3,:) = zeros(size(time));
            secondz(1,4,:) = zeros(size(time));
            secondz(1,5,:) = (-30/d^2).*(3*tau.^2-8*tau.^3+5*tau.^4);
            secondz(2,1,:) = (60*Az/d^3).*(3*tau-12*tau.^2+10*tau.^3);
            secondz(2,2,:) = (60*Az/d^3).*(1-6*tau+6*tau.^2);
            secondz(2,3,:) = zeros(size(time));
            secondz(2,4,:) = zeros(size(time));
            secondz(2,5,:) = (-60/d^2).*(tau-3*tau.^2+2*tau.^3);
            secondz(3,1,:) = (-30/d^2).*(3*tau.^2-8*tau.^3+5*tau.^4);
            secondz(3,2,:) = (30/d^2).*(-2*tau+6*tau.^2-4*tau.^3);
            secondz(3,3,:) = zeros(size(time));
            secondz(3,4,:) = zeros(size(time));
            secondz(3,5,:) = zeros(size(time));
            secondz(4,1,:) = zeros(size(time));
            secondz(4,2,:) = zeros(size(time));
            secondz(4,3,:) = zeros(size(time));
            secondz(4,4,:) = zeros(size(time));
            secondz(4,5,:) = zeros(size(time));
            secondz(5,1,:) = zeros(size(time));
            secondz(5,2,:) = zeros(size(time));
            secondz(5,3,:) = zeros(size(time));
            secondz(5,4,:) = zeros(size(time));
            secondz(5,5,:) = zeros(size(time));
            
            secondtang(1,1,:) = (60/d^3)*sqrt(Ax^2+Ay^2+Az^2).*(15*tau.^4-20*tau.^3+6*tau.^2);
            secondtang(1,2,:) = (-60/d^3)*sqrt(Ax^2+Ay^2+Az^2).*(-10*tau.^3+12*tau.^2-3*tau);
            secondtang(1,3,:) = (-30/d^2)*Ax*1/(sqrt(Ax^2+Ay^2+Az^2)).*(5*tau.^4-8*tau.^3+3*tau.^2);
            secondtang(1,4,:) = (-30/d^2)*Ay*1/(sqrt(Ax^2+Ay^2+Az^2)).*(5*tau.^4-8*tau.^3+3*tau.^2);
            secondtang(1,5,:) = (-30/d^2)*Az*1/(sqrt(Ax^2+Ay^2+Az^2)).*(5*tau.^4-8*tau.^3+3*tau.^2);
            secondtang(2,1,:) = (60/d^3)*sqrt(Ax^2+Ay^2+Az^2).*(10*tau.^3-12*tau.^2+3*tau);
            secondtang(2,2,:) = (-60/d^3)*sqrt(Ax^2+Ay^2+Az^2).*(-6*tau.^2+6*tau-1);
            secondtang(2,3,:) = (-60/d^2)*Ax*1/sqrt(Ax^2+Ay^2+Az^2).*(2*tau.^3-3*tau.^2+tau);
            secondtang(2,4,:) = (-60/d^2)*Ay*1/sqrt(Ax^2+Ay^2+Az^2).*(2*tau.^3-3*tau.^2+tau);
            secondtang(2,5,:) = (-60/d^2)*Az*1/sqrt(Ax^2+Ay^2+Az^2).*(2*tau.^3-3*tau.^2+tau);
            secondtang(3,1,:) = (-30/d^2)*Ax*1/sqrt(Ax^2+Ay^2+Az^2).*(5*tau.^4-8*tau.^3+3*tau.^2);
            secondtang(3,2,:) = (-60/d^2)*Ax*1/sqrt(Ax^2+Ay^2+Az^2).*(2*tau.^3-3*tau.^2+tau);
            secondtang(3,3,:) = (30/d)*(1/sqrt(Ax^2+Ay^2+Az^2))*(1-Ax^2/(Ax^2+Ay^2+Az^2)).*(tau.^4-2*tau.^3+tau.^2);
            secondtang(3,4,:) = (-30/d)*Ax*Ay*1/sqrt((Ax^2+Ay^2+Az^2)^3).*(tau.^4-2*tau.^3+tau.^2);
            secondtang(3,5,:) = (-30/d)*Ax*Az*1/sqrt((Ax^2+Ay^2+Az^2)^3).*(tau.^4-2*tau.^3+tau.^2);
            secondtang(4,1,:) = (-30/d^2)*Ay*1/sqrt(Ax^2+Ay^2+Az^2).*(5*tau.^4-8*tau.^3+3*tau.^2);
            secondtang(4,2,:) = (-60/d^2)*Ay*1/sqrt(Ax^2+Ay^2+Az^2).*(2*tau.^3-3*tau.^2+tau);
            secondtang(4,3,:) = (-30/d)*Ax*Ay*1/sqrt((Ax^2+Ay^2+Az^2)^3).*(tau.^4-2*tau.^3+tau.^2);
            secondtang(4,4,:) = (30/d)*(1/sqrt(Ax^2+Ay^2+Az^2))*(1-Ay^2/(Ax^2+Ay^2+Az^2)).*(tau.^4-2*tau.^3+tau.^2);
            secondtang(4,5,:) = (-30/d)*Az*Ay*1/sqrt((Ax^2+Ay^2+Az^2)^3).*(tau.^4-2*tau.^3+tau.^2);
            secondtang(5,1,:) = (-30/d^2)*Az*1/sqrt(Ax^2+Ay^2+Az^2).*(5*tau.^4-8*tau.^3+3*tau.^2);
            secondtang(5,2,:) = (-60/d^2)*Az*1/sqrt(Ax^2+Ay^2+Az^2).*(2*tau.^3-3*tau.^2+tau);
            secondtang(5,3,:) = (-30/d)*Ax*Az*1/sqrt((Ax^2+Ay^2+Az^2)^3).*(tau.^4-2*tau.^3+tau.^2);
            secondtang(5,4,:) = (-30/d)*Az*Ay*1/sqrt((Ax^2+Ay^2+Az^2)^3).*(tau.^4-2*tau.^3+tau.^2);
            secondtang(5,5,:) = (30/d)*(1/sqrt(Ax^2+Ay^2+Az^2))*(1-Az^2/(Ax^2+Ay^2+Az^2)).*(tau.^4-2*tau.^3+tau.^2);
            
        end
    end
end
