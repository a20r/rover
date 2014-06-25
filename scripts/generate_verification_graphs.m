
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
    subplot(2, 2, 1);
    hold on;
    plot(data(:, 1), data(:, x_e), 'bo', 'linewidth', 1);
    plot(data(:, 1), data(:, x_r), 'r+', 'linewidth', 1);
    ylim([0 600]);
    title("X");

    subplot(2, 2, 2);
    hold on;
    plot(data(:, 1), data(:, y_e), 'bo', 'linewidth', 1);
    plot(data(:, 1), data(:, y_r), 'r+', 'linewidth', 1);
    ylim([0 600]);
    title("Y");

    subplot(2, 2, 3);
    hold on;
    plot(data(:, 1), data(:, z_e), 'r+', 'linewidth', 1);
    plot(data(:, 1), data(:, z_r), 'bo', 'linewidth', 1);
    ylim([0 300]);
    title("Z");

    subplot(2, 2, 4);
    hold on;
    plot(data(:, 1), data(:, b_e), 'bo', 'linewidth', 4);
    plot(data(:, 1), data(:, b_r), 'r+', 'linewidth', 4);
    title("Beta");

    figure;
    plot(data(:, x_e), data(:, y_e), 'bo', 'linewidth', 2);
    hold on;
    plot(data(:, x_r), data(:, y_r), 'r+', 'linewidth', 2);
    ylim([0, 600]);
    xlim([0, 600]);

endfunction

function run()
    file_name = 'sandbox/all_verify.txt';
    data = load(file_name);
    plot_verification(data);
    waitForEnd();
endfunction

run()

