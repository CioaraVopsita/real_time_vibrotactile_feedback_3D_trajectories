for i = 1:50
    peak_cell = PeakStruct{i};
    if size(peak_cell,1)==1
        peaks_amplitude(i)=peak_cell(3);
        peaks_location(i) = peak_cell(4);
    else
        [ii,jj]=find(peak_cell==max(peak_cell(:,3)));
        peaks_amplitude(i)=peak_cell(ii,3);
        peaks_location(i) = peak_cell(ii,4);
    end

end
%%
for i=1:50
    [cost, gradient, hessian, Ftang, Gtang, Ftang2, xpredicted, ypredicted, zpredicted] = submovement(optimization(:,:,i), time, no_submovements, Gx, Gy, Gz);
    figure(i)
    plot(Gtang)
    hold on
    plot(Ftang)
end