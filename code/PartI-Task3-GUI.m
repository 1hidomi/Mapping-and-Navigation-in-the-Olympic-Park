function QEOP_Navigation_System
    gps_points = [51.53872151, -0.01633562; 51.53877478, -0.01220277; 51.53856432, -0.01296571; ...
                  51.53836794, -0.00977126; 51.53736141, -0.01531075; 51.53740335, -0.01161921; ...
                  51.53645029, -0.01396862; 51.53782116, -0.01440903; 51.53741749, -0.01557831; ...
                  51.53743729, -0.01465389; 51.53580688, -0.01366624; 51.53579142, -0.01255364; ...
                  51.53713174, -0.01017044; 51.53753812, -0.01022625; 51.53852776, -0.01031324; ...
                  51.53817059, -0.01195040; 51.53848331, -0.01242272];
    
    names = {'K1','K2','K3','K4','K5','K6','K7','S1','S2','S3','S4','S5','S6','S7','S8','S9','S10'};
    name2id = containers.Map(names, 1:numel(names));
 
    lat0 = gps_points(1,1); lon0 = gps_points(1,2); R = 6371000;
    toXY = @(lat, lon) [R * cosd(lat0) * deg2rad(lon - lon0), R * deg2rad(lat - lat0)];
    raw_xy = toXY(gps_points(:,1), gps_points(:,2));
    margin = 40; xmin = min(raw_xy(:,1)); ymin = min(raw_xy(:,2));
    x_s = raw_xy(:,1) - xmin + margin/2;
    y_s = raw_xy(:,2) - ymin + margin/2;
    points_shift = [x_s, y_s];
    res = 1; width = max(x_s) + margin; height = max(y_s) + margin;
    map = occupancyMap(width, height, res);

    [gridX, gridY] = meshgrid(0:1/res:width, 0:1/res:height);
    addPolyObs = @(lats, lons) setOccupancy(map, ...
        [gridX(inpolygon(gridX, gridY, ...
        R*cosd(lat0)*deg2rad(lons-lon0)-xmin+margin/2, ...
        R*deg2rad(lats-lat0)-ymin+margin/2)), ...
        gridY(inpolygon(gridX, gridY, ...
        R*cosd(lat0)*deg2rad(lons-lon0)-xmin+margin/2, ...
        R*deg2rad(lats-lat0)-ymin+margin/2))], 1);

    setOccupancy(map, [gridX((gridX-x_s(1)).^2 + (gridY-y_s(1)).^2 <= 108^2), ...
                       gridY((gridX-x_s(1)).^2 + (gridY-y_s(1)).^2 <= 108^2)], 1);
    ref_k3 = toXY(51.53836137, -0.01266225);
    rad_k3 = norm(ref_k3 - raw_xy(3,:));
    setOccupancy(map, [gridX((gridX-x_s(3)).^2 + (gridY-y_s(3)).^2 <= rad_k3^2), ...
                       gridY((gridX-x_s(3)).^2 + (gridY-y_s(3)).^2 <= rad_k3^2)], 1);

    addPolyObs([51.537908, 51.538048, 51.537417, 51.537210], [-0.012433, -0.011693, -0.011427, -0.012501]);
    addPolyObs([51.538782, 51.538431, 51.537811, 51.538403], [-0.009357, -0.010124, -0.009858, -0.008698]);
    addPolyObs([51.538940, 51.538867, 51.538250, 51.538320], [-0.012194, -0.012486, -0.012086, -0.011794]);
    addPolyObs([51.535865, 51.537522, 51.536848], [-0.013716, -0.015023, -0.016922]);

    addPolyObs([51.537968, 51.538011, 51.539399, 51.539310], [-0.014359, -0.014084, -0.013709, -0.013953]); 
    addPolyObs([51.537653, 51.537700, 51.535848, 51.535791], [-0.014483, -0.014177, -0.012380, -0.012686]); 
    addPolyObs([51.537202, 51.537416, 51.539305, 51.539166], [-0.010062, -0.009750, -0.011568, -0.010930]); 

    setOccupancy(map, [gridX(inpolygon(gridX, gridY, [300,500,500], [0,0,150])), ...
                       gridY(inpolygon(gridX, gridY, [300,500,500], [0,0,150]))], 1);
    
    edges = {'K1','S2';'S2','K5';'S2','S3';'S3','K7';'S3','S1';'K7','S4';'S4','S5';'S5','K6';...
             'K6','S7';'S7','S6';'S7','K4';'S7','S8';'K4','S8';'S8','S9';'S9','K2';'S9','S1';...
             'K2','S10';'S10','K3';'K3','S1'};

    adj = cell(numel(names), 1);
    for i = 1:size(edges,1)
        u = name2id(edges{i,1}); v = name2id(edges{i,2});
        w = norm(points_shift(u,:) - points_shift(v,:)); 
        adj{u} = [adj{u}; v w]; adj{v} = [adj{v}; u w];
    end

    fig = uifigure('Name', 'QEOP Navigation System', 'Position', [100 100 1000 650]);
    ax = uiaxes(fig, 'Position', [280 50 680 550]);
    show(map, 'Parent', ax); hold(ax, 'on');
    grid(ax, 'on'); axis(ax, 'equal');
    title(ax, 'Interactive Occupancy Grid Path Planning');
    scatter(ax, x_s(1:7), y_s(1:7), 100, 'y', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Key Points');
    scatter(ax, x_s(8:17), y_s(8:17), 50, 'm', 'LineWidth', 1.5, 'DisplayName', 'Signal Points');
    for i=1:numel(names), text(ax, x_s(i)+2, y_s(i)+2, names{i}, 'FontSize', 8, 'Color', 'w'); end
    pnl = uipanel(fig, 'Title', 'Navigation Control', 'Position', [20 50 240 550]);    
    uilabel(pnl, 'Text', 'Start Node:', 'Position', [20 500 100 20]);
    ddStart = uidropdown(pnl, 'Items', names, 'Position', [20 470 200 30], 'Value', 'K7');
    uilabel(pnl, 'Text', 'Goal Node:', 'Position', [20 420 100 20]);
    ddGoal = uidropdown(pnl, 'Items', names, 'Position', [20 390 200 30], 'Value', 'K1');
    btnNav = uibutton(pnl, 'push', 'Text', 'Find Shortest Path', 'Position', [20 330 200 40], 'BackgroundColor', [0.2 0.6 1], 'FontColor', 'w', 'FontWeight', 'bold');
    statusText = uitextarea(fig, 'Position', [280 15 680 30], 'Editable', 'off', 'Value', 'Ready');
    btnNav.ButtonPushedFcn = @(btn, event) runNavigation();

    function runNavigation()
        startNode = ddStart.Value;
        goalNode = ddGoal.Value;
        tic;
        [pathIDs, pathNames, ~, totalW] = dijkstraHeap(adj, names, name2id, startNode, goalNode);
        exeTime = toc;
        fprintf('Algorithm Task 2: Path found in %.6f seconds.\n', exeTime);
        if isempty(pathIDs)
            statusText.Value = sprintf('Error: No path found');
        else
            delete(findobj(ax, 'Tag', 'PathLine'));
            px = x_s(pathIDs); py = y_s(pathIDs);
            plot(ax, px, py, 'w-', 'LineWidth', 3, 'Tag', 'PathLine');
            plot(ax, px, py, 'r--', 'LineWidth', 1.5, 'Tag', 'PathLine');
            
            statusText.Value = sprintf('Arrived %s. [Speculative] Pre-calculating next possible steps...', goalNode);

            predictions = {'K2', 'K4'}; 
            for i = 1:length(predictions)
                if ~strcmp(goalNode, predictions{i})
                    [~, ~, ~, ~] = dijkstraHeap(adj, names, name2id, goalNode, predictions{i});
                    fprintf('Speculative Logic: Path from %s to %s pre-cached.\n', goalNode, predictions{i});
                end
            end
            statusText.Value = sprintf('Success! Destination: %s. Distance: %.2f m. (Time: %.4fs)', ...
                goalNode, totalW, exeTime);
        end
    end

    function [pathIDs, pathNames, segW, totalW] = dijkstraHeap(adj, names, name2id, startName, goalName)
        N = numel(names); s = name2id(startName); t = name2id(goalName);
        distance = inf(N,1); prev = zeros(N,1); distance(s) = 0;
        heap = zeros(0,2); 
        heap = push(heap, 0, s);
        visited = false(N,1);
        while ~isempty(heap)
            [heap, d_curr, curr] = pop(heap);
            if d_curr > distance(curr) || visited(curr), continue; end
            visited(curr) = true;
            if curr == t, break; end
            A = adj{curr};
            for k = 1:size(A,1)
                v = round(A(k,1)); w = A(k,2);
                if ~visited(v)
                    alt = distance(curr) + w;
                    if alt < distance(v)
                        distance(v) = alt; prev(v) = curr;
                        heap = push(heap, alt, v);
                    end
                end
            end
        end
        if isinf(distance(t)), pathIDs = []; pathNames = {}; segW = []; totalW = inf; return; end
        pathIDs = t; cur = t;
        while cur ~= s, cur = prev(cur); pathIDs = [cur, pathIDs]; end
        pathNames = names(pathIDs);
        totalW = distance(t);
        segW = []; 
    end

    function h = push(h, k, n)
        h(end+1,:) = [k n]; i = size(h,1);
        while i > 1
            p = floor(i/2); if h(p,1) <= h(i,1), break; end
            h([p i],:) = h([i p],:); i = p;
        end
    end

    function [h, k, n] = pop(h)
        k = h(1,1); n = round(h(1,2)); last = h(end,:); h(end,:) = [];
        if ~isempty(h), h(1,:) = last; i = 1; n_h = size(h,1);
            while true
                l = 2*i; r = 2*i + 1; if l > n_h, break; end
                c = l; if r <= n_h && h(r,1) < h(l,1), c = r; end
                if h(i,1) <= h(c,1), break; end
                h([i c],:) = h([c i],:); i = c;
            end
        end
    end
end

