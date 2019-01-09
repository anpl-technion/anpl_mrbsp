function  P = writepoints( points, filename )
%WRITEPOINTS Write 3D points to a file
% Points are given as a 3xN matrix

file = fopen(filename, 'w');
N = size(points,2)
fprintf(file, '%d\n', N);
P = fprintf(file, '%f%f%f\n', points);
fclose(file);

end

