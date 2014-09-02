
function waitForEnd()
    for i = 1:10000
        k = waitforbuttonpress;
        if k == 1
            break;
        end
    end
endfunction

function plot_verification(data)
    x_e = 2;
    y_e = 3;
    z_e = 4;
    b_e = 5;

    x_r = 6;
    y_r = 7;
    z_r = 8;
    b_r = 9;

    x = data(:, x_e);
    y = data(:, y_e);
    z = data(:, z_e);

    figure;
    risk = load('sandbox/risk.out');
    risk = reshape(risk, 100, 100);
    risk = risk';
    xi = 0.1:0.1:10;
    yi = 0.1:0.1:10;
    [X Y] = meshgrid(xi, yi);
    nq = 2;
    pcolor(X, Y, risk);
    % surf(X, Y, risk);
    shading interp;
    colormap("jet");
    hold on;
    plot3(x(1:nq:end) ./ 100, y(1:nq:end) ./ 100, z(1:nq:end) ./ 100, 'r', 'linewidth', 2);
    plot3(x(2:nq:end) ./ 100, y(2:nq:end) ./ 100, z(2:nq:end) ./ 100, 'b', 'linewidth', 2);
    view(314, 36);
    box off;
    xlim([-0.5 10.5]);
    ylim([-0.5 10.5]);
    xlabel("x [m]", 'FontSize', 15, 'fontname', 'Helvetica');
    ylabel("y [m]", 'FontSize', 15, 'fontname', 'Helvetica');
    zlabel("z [m]", 'FontSize', 15, 'fontname', 'Helvetica');
    colorbar('FontSize', 15, 'fontname', 'Helvetica');
    set(gca, 'fontsize', 15, 'fontname', 'Helvetica');
    % print('../SSCI14-PARCov/tasefigs/trajpractical.png', '-dpng');

    figure;
    risk(1, 1) = 1;
    pcolor(X, Y, risk);
    colormap("jet");
    shading interp;
    xlabel("x position [m]", 'FontSize', 15, 'fontname', 'Helvetica');
    ylabel("y position [m]", 'FontSize', 15, 'fontname', 'Helvetica');
    set(gca, 'fontsize', 15, 'fontname', 'Helvetica');
    colorbar('FontSize', 15, 'fontname', 'Helvetica');
    xlim([0 10]); ylim([0 10]);
    % print('../SSCI14-PARCov/tasefigs/s-1000-3.png', '-dpng');
    % print("sandbox/heatmap.png", "-dpng");

    figure;
    xi = 0:10:999;
    yi = 0:10:999;
    [X Y] = meshgrid(xi, yi);
    grid = load('sandbox/grids/1.out');
    grid = grid'';
    m = max(max(grid));
    pcolor(X, Y, m - grid);
    xlabel("X"); ylabel("Y");
    colorbar;
    shading interp;

    print("sandbox/grid.png", "-dpng");

    k = 1;
    for i=[1 25 75 100]
        figure;
        xi = 0.1:0.1:10;
        yi = 0.1:0.1:10;
        [X Y] = meshgrid(xi, yi);
        grid = load(strcat('sandbox/grids/', num2str(i), '.out'));
        % grid = load('sandbox/grid.out');
        grid = grid'';
        m = max(max(grid));
        grid(find(grid == m)) = m - 1;
        pcolor(X, Y, m - 1 - grid);
        colorbar;
        shading interp;
        xlabel("x position [m]", 'FontSize', 13, 'fontname', 'Helvetica');
        ylabel("y position [m]", 'FontSize', 13, 'fontname', 'Helvetica');
        set(gca, 'fontsize', 15, 'fontname', 'Helvetica');
        colorbar('FontSize', 15, 'fontname', 'Helvetica');
        % print(strcat('../SSCI14-PARCov/tasefigs/grid', num2str(k), '.png'), '-dpng');

        figure;
        cost = 100 .* risk + grid;
        pcolor(X, Y, cost);
        colorbar;
        shading interp;
        xlabel("x position [m]", 'FontSize', 13, 'fontname', 'Helvetica');
        ylabel("y position [m]", 'FontSize', 13, 'fontname', 'Helvetica');
        set(gca, 'fontsize', 15, 'fontname', 'Helvetica');
        colorbar('FontSize', 15, 'fontname', 'Helvetica');
        % print(strcat('../SSCI14-PARCov/tasefigs/cost', num2str(k), '.png'), '-dpng');
        k = k + 1;

    end

endfunction

function run()
    file_name = 'sandbox/multi_verify.txt';
    data = load(file_name);
    plot_verification(data);
    waitForEnd();
endfunction

run()

