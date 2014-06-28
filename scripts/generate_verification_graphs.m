
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
    contour3(X, Y, risk);
    hold on;
    plot3(x(1:nq:end), y(1:nq:end), z(1:nq:end), 'r', 'linewidth', 2);
    plot3(x(2:nq:end), y(2:nq:end), z(2:nq:end), 'b', 'linewidth', 2);
    xlabel("X"); ylabel("Y"); zlabel("Z");
    % plot3(x(3:nq:end), y(3:nq:end), z(3:nq:end), 'g', 'linewidth', 2);

endfunction

function run()
    file_name = 'sandbox/all_verify.txt';
    data = load(file_name);
    plot_verification(data);
    waitForEnd();
endfunction

run()

