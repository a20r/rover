
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

    figure;
    x = data(:, x_e);
    y = data(:, y_e);
    z = data(:, z_e);

    risk = load("sandbox/risk.out");
    risk = reshape(risk, 100, 100);
    risk = risk';
    xi = 0:10:999;
    yi = 0:10:999;
    [X Y] = meshgrid(xi, yi);
    nq = 2;
    % contour3(X, Y, risk);
    pcolor(X, Y, risk);
    shading interp;
    colormap("pink");
    colorbar;
    % colorbar;
    hold on;
    plot3(x(1:nq:end), y(1:nq:end), z(1:nq:end), 'r', 'linewidth', 2);
    plot3(x(2:nq:end), y(2:nq:end), z(2:nq:end), 'b', 'linewidth', 2);
    view(314, 36);
    xlabel("X"); ylabel("Y"); zlabel("Z"); box off;
    print("sandbox/trajectories.png", "-dpng");

    figure;
    pcolor(X, Y, risk);
    colormap("pink");
    shading interp;
    colorbar;
    print("sandbox/heatmap.png", "-dpng");

    figure;
    xi = 0:4:999;
    yi = 0:4:999;
    [X Y] = meshgrid(xi, yi);
    grid = load("sandbox/grid.out");
    grid = grid'';
    pcolor(X,Y,grid);
    xlabel("X"); ylabel("Y");
    colorbar;
    shading interp;
    print("sandbox/grid.png", "-dpng");

endfunction

function run()
    file_name = 'sandbox/all_verify.txt';
    data = load(file_name);
    plot_verification(data);
    waitForEnd();
endfunction

run()

