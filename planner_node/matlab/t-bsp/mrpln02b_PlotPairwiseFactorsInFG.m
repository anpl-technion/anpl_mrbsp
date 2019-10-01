function h = mrpln02b_PlotPairwiseFactorsInFG(fg, values, plot_flags)

factor_color = plot_flags.factor_color;
factor_line_width = plot_flags.factor_line_width;
factor_linestyle  = plot_flags.factor_linestyle;
h = []; odometryPathXCoords = []; odometryPathYCoords = [];
for i=0:double(fg.size)-1
    if ~fg.exists(i), continue; end
    f = fg.at(i);
    keys   = f.keys();
    delta = abs(double(gtsam.mrsymbolIndex(keys.front()))-double(gtsam.mrsymbolIndex(keys.back())));
    if delta == 1   
        % continue; % skip odometry factors for better visibility
        factor_linestyle = '-';
    else
        continue % skip LC factors
        factor_linestyle = '--';
    end
    
    edge = mrpln02c_PlotPairwiseFactor(f, values, factor_color, factor_line_width, factor_linestyle);
    h = [h edge];
    if delta == 1
        odometryPathXCoords = [odometryPathXCoords edge.XData];
        odometryPathYCoords = [odometryPathYCoords edge.YData];
        hold on; plot(edge.XData, edge.YData, 'Color', plot_flags.factor_color, 'LineWidth', plot_flags.factor_line_width+2)
    end
    
end


hold on; plot(odometryPathXCoords, odometryPathYCoords, '.', 'Color', plot_flags.factor_color, 'MarkerSize', 30) % mark observation poses
