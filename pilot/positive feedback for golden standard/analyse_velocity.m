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