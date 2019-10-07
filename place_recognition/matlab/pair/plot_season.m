close all seasons = {'Spring';
'Summer';
'Fall';
'Winter';
}
;
data = [
  0.774,        0.651, 0.482, 0.789, 0.907, 0.953, 0.256,
  0.932; 0.736, 0.700, 0.583, 0.560, 0.864, 0.957, 0.122,
         0.918; 0.589, 0.611, 0.427, 0.309, 0.533, 0.771, 0.047,
                0.679; 0.419, 0.351, 0.537, 0.035, 0.317, 0.641, 0.170,
                       0.778; 0.764, 0.672, 0.640, 0.702, 0.893, 0.923, 0.081,
                              0.914; 0.557, 0.496, 0.259, 0.466, 0.605, 0.789, 0.138,
                                     0.694; 0.489, 0.379, 0.512, 0.019, 0.305, 0.576, 0.256,
                                            0.738; 0.599, 0.454, 0.003, 0.348, 0.557, 0.656, 0.002,
                                                   0.003; 0.443, 0.351, 0.078, 0.004, 0.360, 0.525, 0.139,
                                                          0.606; 0.597, 0.491, 0.158, 0.014, 0.627, 0.808, 0.002,
                                                                 0.000;
];
colors = [ 'y', 'm', 'c', 'r', 'g', 'b', 'w', 'k' ];

for
  i = 1 : 8 % plot(1 : 4, data(1 : 4, i), strcat(colors(i), '-'));
hold on;
plot(5 : 7, data(5 : 7, i), strcat(colors(i), '-'));
hold on;
% plot(8 : 9, data(8 : 9, i), strcat(colors(i), '-'));
hold on;
plot(10, data(10, i), strcat(colors(i), '-'));
hold on;
plot(1 : 4, data(1 : 4, i), strcat(colors(i), '-*'));
hold on;
end axis([0 5 0 1]) set(gca, 'xtick', 1 : 4, 'xticklabel', seasons)
    set(gca, 'Color', [0.7 0.7 0.7]) grid on
    title('AUCs with different seasons (Spring as query)')
        legend(colors,
               {'CNN', 'NBLD', 'NetV.', 'DELI.', 'M2DP', 'S.C.', 'BoW', 'GIST'})