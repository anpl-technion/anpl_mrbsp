function h = mrpln02c_PlotPairwiseFactor(factor, values, factor_color, factor_line_width, factor_linestyle)

h = [];
if nargin < 3
    factor_color = 'yellow';
    factor_line_width = 5;
    factor_linestyle = '-';
end

keys = factor.keys();
if keys.size() ~= 2, return; end;

% Extract values for keys
value1 = values.at(keys.at(0));
value2 = values.at(keys.at(1));

N1 = value1.x;
E1 = value1.y;
N2 = value2.x;
E2 = value2.y;

%h = line([E1; E2], [N1; N2], 'Color',factor_color, 'LineWidth',factor_line_width,'LineStyle',factor_linestyle);
h = line([N1; N2], [E1; E2], 'Color',factor_color, 'LineWidth',factor_line_width,'LineStyle',factor_linestyle);