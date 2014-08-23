
function waitForEnd()
    for i = 1:10000
        k = waitforbuttonpress;
        if k == 1
            break;
        end
    end
endfunction

function [data_1000 data_1500 data_2000] = loadDir(dir, index)
    nqs = [1 5 10 15 20 25 30 35];
    data_1000 = zeros(1, length(nqs));
    data_1500 = zeros(1, length(nqs));
    data_2000 = zeros(1, length(nqs));
    for i = 1:length(nqs)
        data_1000_1 = load(strcat(dir, 'out_s-1000-1_90_0_', num2str(nqs(i)), '.txt'));
        data_1000_2 = load(strcat(dir, 'out_s-1000-2_90_0_', num2str(nqs(i)), '.txt'));
        data_1000_3 = load(strcat(dir, 'out_s-1000-3_90_0_', num2str(nqs(i)), '.txt'));

        data_1000_1(isnan(data_1000_1)) = 0;
        data_1000_2(isnan(data_1000_2)) = 0;
        data_1000_3(isnan(data_1000_3)) = 0;
        data_1000(i) = mean([mean(data_1000_1)(index) mean(data_1000_2)(index) mean(data_1000_3)(index)]);

        data_1500_1 = load(strcat(dir, 'out_s-1500-1_90_0_', num2str(nqs(i)), '.txt'));
        data_1500_2 = load(strcat(dir, 'out_s-1500-2_90_0_', num2str(nqs(i)), '.txt'));
        data_1500_3 = load(strcat(dir, 'out_s-1500-3_90_0_', num2str(nqs(i)), '.txt'));

        data_1500_1(isnan(data_1500_1)) = 0;
        data_1500_2(isnan(data_1500_2)) = 0;
        data_1500_3(isnan(data_1500_3)) = 0;
        data_1500(i) = mean([mean(data_1500_1)(index) mean(data_1500_2)(index) mean(data_1500_3)(index)]);

        data_2000_1 = load(strcat(dir, 'out_s-2000-1_90_0_', num2str(nqs(i)), '.txt'));
        data_2000_2 = load(strcat(dir, 'out_s-2000-2_90_0_', num2str(nqs(i)), '.txt'));
        data_2000_3 = load(strcat(dir, 'out_s-2000-3_90_0_', num2str(nqs(i)), '.txt'));

        data_2000_1(isnan(data_2000_1)) = 0;
        data_2000_2(isnan(data_2000_2)) = 0;
        data_2000_3(isnan(data_2000_3)) = 0;
        data_2000(i) = mean([mean(data_2000_1)(index) mean(data_2000_2)(index) mean(data_2000_3)(index)]);
    end
endfunction

function graphPerfQuads()
    nqs = [1 5 10 15 20 25 30 35];
    [data_1000 data_1500 data_2000] = loadDir('experiments/latest/PerfQuads/data/', 7);
    figure;
    plot(nqs, data_1000, 'bd--', 'linewidth', 2, 'markersize', 10);
    hold on;
    plot(nqs, data_1500, 'rd--', 'linewidth', 2, 'markersize', 10);
    plot(nqs, data_2000, 'gd--', 'linewidth', 2, 'markersize', 10);
    title('Number of iterations needed to reach 90% cumulative coverage');
    legend('1000x1000', '1500x1500', '2000x2000');
    xlabel('Number of Quads');
    ylabel('Number of Iterations');

    [risk_1000 risk_1500 risk_2000] = loadDir('experiments/latest/PerfQuads/data/', 3);
    [sq_1000 sq_1500 sq_2000] = loadDir('experiments/latest/PerfQuads/data/', 2);

    figure;
    plot(nqs, risk_1000, 'bd--', 'linewidth', 2, 'markersize', 10);
    hold on;
    plot(nqs, risk_1500, 'rd--', 'linewidth', 2, 'markersize', 10);
    plot(nqs, risk_2000, 'gd--', 'linewidth', 2, 'markersize', 10);

    plot(nqs, sq_1000, 'bo--', 'markersize', 10, 'linewidth', 2);
    plot(nqs, sq_1500, 'ro--', 'markersize', 10, 'linewidth', 2);
    plot(nqs, sq_2000, 'go--', 'markersize', 10, 'linewidth', 2);

    title('Sensor quality and Risk');
    legend(
        'Risk 1000x1000', 'Risk 1500x1500', 'Risk 2000x2000',
        'SQ 1000x1000', 'SQ 1500x1500', 'SQ 2000x2000', -1
    );
    xlabel('Number of Quads');
    ylabel('%');

endfunction



graphPerfQuads();
waitForEnd();
