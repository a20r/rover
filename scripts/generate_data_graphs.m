
function waitForEnd()
    for i = 1:10000
        k = waitforbuttonpress;
        if k == 1
            break;
        end
    end
endfunction


function data = loadData(dir, dim, h, cn, nq, index)
    data_1 = load(strcat(dir, 'out_s-', num2str(dim), '-1_', num2str(h), '_', num2str(cn), '_', num2str(nq), '.txt'));
    data_2 = load(strcat(dir, 'out_s-', num2str(dim), '-2_', num2str(h), '_', num2str(cn), '_', num2str(nq), '.txt'));
    data_3 = load(strcat(dir, 'out_s-', num2str(dim), '-3_', num2str(h), '_', num2str(cn), '_', num2str(nq), '.txt'));

    data = mean([mean(data_1)(index) mean(data_2)(index) mean(data_3)(index)]);
endfunction


function timeData = loadTimeData(dir, dim, h, cn, nq)

    data_1 = load(strcat(dir, 'out_s-', num2str(dim), '-1_', num2str(h), '_', num2str(cn), '_', num2str(nq), '.txt'));
    data_2 = load(strcat(dir, 'out_s-', num2str(dim), '-2_', num2str(h), '_', num2str(cn), '_', num2str(nq), '.txt'));
    data_3 = load(strcat(dir, 'out_s-', num2str(dim), '-3_', num2str(h), '_', num2str(cn), '_', num2str(nq), '.txt'));

    timeData = mean([
        data_1(:, 10)(750) - data_1(:, 10)(1)
        data_2(:, 10)(750) - data_2(:, 10)(1)
        data_3(:, 10)(750) - data_3(:, 10)(1)]);

endfunction


function [data_1000 data_1500 data_2000] = loadQuadData(dir, index)
    nqs = [1 5 10 15 20 25 30 35];
    data_1000 = zeros(1, length(nqs));
    data_1500 = zeros(1, length(nqs));
    data_2000 = zeros(1, length(nqs));
    for i = 1:length(nqs)
        data_1000(i) = loadData(dir, 1000, 90, 0, nqs(i), index);
        data_1500(i) = loadData(dir, 1500, 90, 0, nqs(i), index);
        data_2000(i) = loadData(dir, 2000, 90, 0, nqs(i), index);
    end
endfunction


function data = loadControlNoiseData(dir, nq, index)
    cns = [0, 20, 40, 60];
    data = zeros(1, length(cns));
    for i = 1:length(cns)
        data(i) = loadData(dir, 1500, 90, cns(i), nq, index);
    end
endfunction


function graphPerfQuads()
    nqs = [1 5 10 15 20 25 30 35];
    [data_1000 data_1500 data_2000] = loadQuadData('experiments/latest/PerfQuads/data/', 7);
    figure;
    plot(nqs, data_1000, 'bo--', 'linewidth', 2, 'markersize', 10);
    hold on;
    plot(nqs, data_1500, 'ro--', 'linewidth', 2, 'markersize', 10);
    plot(nqs, data_2000, 'go--', 'linewidth', 2, 'markersize', 10);
    % title('Number of iterations needed to reach 90% cumulative coverage');
    legend('1000x1000', '1500x1500', '2000x2000');
    xlabel('Number of quads');
    ylabel('Number of iterations');

    [risk_1000 risk_1500 risk_2000] = loadQuadData('experiments/latest/PerfQuads/data/', 3);
    [sq_1000 sq_1500 sq_2000] = loadQuadData('experiments/latest/PerfQuads/data/', 9);

    figure;
    plot(nqs, sq_1000, 'bo--', 'linewidth', 2, 'markersize', 10);
    hold on;
    plot(nqs, sq_1500, 'ro--', 'linewidth', 2, 'markersize', 10);
    plot(nqs, sq_2000, 'go--', 'linewidth', 2, 'markersize', 10);

    plot(nqs, risk_1000, 'bx--', 'markersize', 10, 'linewidth', 2);
    plot(nqs, risk_1500, 'rx--', 'markersize', 10, 'linewidth', 2);
    plot(nqs, risk_2000, 'gx--', 'markersize', 10, 'linewidth', 2);

    % title('Risk and sensor quality as a function of the number of quads');
    legend(
        'SQ 1000x1000', 'SQ 1500x1500', 'SQ 2000x2000',
        'Risk 1000x1000', 'Risk 1500x1500', 'Risk 2000x2000', -1
    );
    xlabel('Number of quads');
    ylabel('%');

endfunction


function graphPerfControlNoise()
    cns = [0, 20, 40, 60];
    data_10 = loadControlNoiseData('experiments/latest/PerfControlNoise/data/', 10, 7);
    data_20 = loadControlNoiseData('experiments/latest/PerfControlNoise/data/', 20, 7);
    data_30 = loadControlNoiseData('experiments/latest/PerfControlNoise/data/', 30, 7);

    figure;
    plot(cns, data_10, 'bo--', 'linewidth', 2, 'markersize', 10);
    hold on;
    plot(cns, data_20, 'ro--', 'linewidth', 2, 'markersize', 10);
    plot(cns, data_30, 'go--', 'linewidth', 2, 'markersize', 10);

    % title('Control noise and cumulative convergence');
    legend('10 quads', '20 quads', '30 quads');
    xlabel('Standard deviation of control error (cm)');
    ylabel('Number of iterations');

    risk_10 = loadControlNoiseData('experiments/latest/PerfControlNoise/data/', 10, 3);
    risk_20 = loadControlNoiseData('experiments/latest/PerfControlNoise/data/', 20, 3);
    risk_30 = loadControlNoiseData('experiments/latest/PerfControlNoise/data/', 30, 3);

    sq_10 = loadControlNoiseData('experiments/latest/PerfControlNoise/data/', 10, 9);
    sq_20 = loadControlNoiseData('experiments/latest/PerfControlNoise/data/', 20, 9);
    sq_30 = loadControlNoiseData('experiments/latest/PerfControlNoise/data/', 30, 9);

    figure;
    plot(cns, sq_10, 'bo--', 'linewidth', 2, 'markersize', 10);
    hold on;
    plot(cns, sq_20, 'ro--', 'linewidth', 2, 'markersize', 10);
    plot(cns, sq_30, 'go--', 'linewidth', 2, 'markersize', 10);

    plot(cns, risk_10, 'bx--', 'linewidth', 2, 'markersize', 10);
    plot(cns, risk_20, 'rx--', 'linewidth', 2, 'markersize', 10);
    plot(cns, risk_30, 'gx--', 'linewidth', 2, 'markersize', 10);

    % title('Risk and sensor quality as a function of control noise');
    legend(
        'SQ 10 quads', 'SQ 20 quads', 'SQ 30 quads',
        'Risk 10 quads', 'Risk 20 quads', 'Risk 30 quads'
    );
    xlabel('Standard deviation of control error (cm)');
    ylabel('Number of iterations');

endfunction

function graphTimeData()

    nqs = [1, 5, 10, 15, 20, 25, 30, 35];
    data = zeros(1, length(nqs));

    for i = 1:length(nqs)
        data(i) = loadTimeData('experiments/latest/PerfQuads/data/', 2000, 90, 0, nqs(i));
    end

    figure;
    plot(nqs, data ./ 750, 'bo--', 'linewidth', 2, 'markersize', 10);
    % title('Computational efficiency of algorithm');
    xlabel('Number of quads');
    ylabel('Seconds per iteration');
    hold on;

endfunction


graphPerfQuads();
graphPerfControlNoise();
graphTimeData();
waitForEnd();
