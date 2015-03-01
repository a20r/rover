
function waitForEnd()
    for i = 1:10000
        k = waitforbuttonpress;
        if k == 1
            break;
        end
    end
endfunction

function graphSandboxData()
    data = load('sandbox/all.txt');
    l = size(data)(1);
    x = 1:l;
    plot(x, data(:, 5), 'b+-', 'linewidth', 4);
    hold on;
    plot(x, data(:, 6), 'g+-', 'linewidth', 4);
    % plot(x, data(:, 7), 'r+-', 'linewidth', 4);
    % plot(x, data(:, 8), 'm+-', 'linewidth', 4);
    % plot(x, data(:, 9), 'c+-', 'linewidth', 4);
    title('Rover Results');
    xlabel('Iteration');
    ylabel('%');
    legend(
        'Cumulative Coverage',
        'Coverage to 90%'
    );
    hold off;

    figure;
    plot(x, data(:, 2), 'g+-', 'linewidth', 4);
    hold on;
    plot(x, data(:, 3), 'r+-', 'linewidth', 4);
    legend('Average Sensor Quality', 'Average Risk');

    figure;
    plot(x, data(:, 7), 'r+', 'linewidth', 4);
    legend('Average Iterations Between Convergences');

    figure;
    plot(x, data(:, 8), 'm+', 'linewidth', 4);
    legend('Average Time Between Convergences');

    figure;
    plot(x, data(:, 9), 'c+-', 'linewidth', 4);
    legend('Performance Metric');

endfunction

graphSandboxData();
waitForEnd();
