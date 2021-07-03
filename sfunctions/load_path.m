


path_data = jsondecode(fileread('../track_data/simple_track.json'));

[N,M] = size(path_data.D);
t = linspace(0, 1*(N-1), N)';

path_data.N = N;

% figure(); hold on;
% plot( path_data.X, path_data.Y )


path.time = t;
path.signals.values = [path_data.D, path_data.X, path_data.Y, path_data.PSI, path_data.K];
path.signals.dimensions = 5;


