
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
    plot(x, data(:, 1), 'b+-', 'linewidth', 4);
    hold on;
    plot(x, data(:, 2), 'g+-', 'linewidth', 4); 
    plot(x, data(:, 3), 'r+-', 'linewidth', 4);
    title('Latest Non-experimental Run Results');
    xlabel('Iteration');
    ylabel('%');
    legend(
        'Sensor coverage',
        'Sensor quality',
        'Risk',
        'Location', 'EastOutside'
    );
    hold off;
endfunction

graphSandboxData();
waitForEnd();
