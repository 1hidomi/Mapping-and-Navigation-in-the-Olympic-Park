gps_points = [
    51.53872151, -0.01633562; % K1 
    51.53877478, -0.01220277; % K2
    51.53856432, -0.01296571; % K3 
    51.53836794, -0.00977126; % K4
    51.53736141, -0.01531075; % K5
    51.53740335, -0.01161921; % K6
    51.53645029, -0.01396862; % K7
    51.53782116, -0.01440903; % S1
    51.53741749, -0.01557831; % S2
    51.53743729, -0.01465389; % S3
    51.53580688, -0.01366624; % S4
    51.53579142, -0.01255364; % S5
    51.53713174, -0.01017044; % S6
    51.53753812, -0.01022625; % S7
    51.53852776, -0.01031324; % S8
    51.53817059, -0.01195040; % S9
    51.53848331, -0.01242272  % S10
];

lat0 = gps_points(1,1); lon0 = gps_points(1,2); R = 6371000;
toXY = @(lat, lon) [R * cosd(lat0) * deg2rad(lon - lon0), R * deg2rad(lat - lat0)];
raw_xy = toXY(gps_points(:,1), gps_points(:,2));
margin = 40;
xmin = min(raw_xy(:,1)); ymin = min(raw_xy(:,2));
x_shift = raw_xy(:,1) - xmin + margin/2;
y_shift = raw_xy(:,2) - ymin + margin/2;
res = 1;
width = max(x_shift) + margin; height = max(y_shift) + margin;
map = occupancyMap(width, height, res);
[gridX, gridY] = meshgrid(0:1/res:width, 0:1/res:height);

addPolyObs = @(lats, lons) setOccupancy(map, ...
    [gridX(inpolygon(gridX, gridY, ...
    R*cosd(lat0)*deg2rad(lons-lon0)-xmin+margin/2, ...
    R*deg2rad(lats-lat0)-ymin+margin/2)), ...
    gridY(inpolygon(gridX, gridY, ...
    R*cosd(lat0)*deg2rad(lons-lon0)-xmin+margin/2, ...
    R*deg2rad(lats-lat0)-ymin+margin/2))], 1);

setOccupancy(map, [gridX((gridX-x_shift(1)).^2 + (gridY-y_shift(1)).^2 <= 108^2), ...
                   gridY((gridX-x_shift(1)).^2 + (gridY-y_shift(1)).^2 <= 108^2)], 1);

ref_k3 = toXY(51.53836137, -0.01266225);
rad_k3 = norm(ref_k3 - raw_xy(3,:));
setOccupancy(map, [gridX((gridX-x_shift(3)).^2 + (gridY-y_shift(3)).^2 <= rad_k3^2), ...
                   gridY((gridX-x_shift(3)).^2 + (gridY-y_shift(3)).^2 <= rad_k3^2)], 1);

addPolyObs([51.537908, 51.538048, 51.537417, 51.537210], [-0.012433, -0.011693, -0.011427, -0.012501]);
addPolyObs([51.538782, 51.538431, 51.537811, 51.538403], [-0.009357, -0.010124, -0.009858, -0.008698]);
addPolyObs([51.538940, 51.538867, 51.538250, 51.538320], [-0.012194, -0.012486, -0.012086, -0.011794]);
addPolyObs([51.535865, 51.537522, 51.536848], [-0.013716, -0.015023, -0.016922]);

addPolyObs([51.537968, 51.538011, 51.539399, 51.539310], [-0.014359, -0.014084, -0.013709, -0.013953]); 
addPolyObs([51.537653, 51.537700, 51.535848, 51.535791], [-0.014483, -0.014177, -0.012380, -0.012686]); 
addPolyObs([51.537202, 51.537416, 51.539305, 51.539166], [-0.010062, -0.009750, -0.011568, -0.010930]); 

setOccupancy(map, [gridX(inpolygon(gridX, gridY, [300,500,500], [0,0,150])), ...
                   gridY(inpolygon(gridX, gridY, [300,500,500], [0,0,150]))], 1);

show(map); hold on;

scatter(x_shift(1:7), y_shift(1:7), 100, 'y', 'filled', 'MarkerEdgeColor', 'k');
for i=1:7, text(x_shift(i)+3, y_shift(i), ['K', num2str(i)], 'Color', 'y', 'FontWeight', 'bold'); end
scatter(x_shift(8:17), y_shift(8:17), 60, 'm', 'LineWidth', 1.5);
for i=1:10, text(x_shift(i+7)+3, y_shift(i+7), ['S', num2str(i)], 'Color', 'm', 'FontSize', 8); end

title('Occupancy Grid Map (Task 1)');
xlabel('Local X (meters)'); ylabel('Local Y (meters)');
axis equal; grid on;
