
function waitForEnd()
    for i = 1:10000
        k = waitforbuttonpress;
        if k == 1
            break;
        end
    end
endfunction

function ret_data = loadDir(dir)
    nqs = [1 5 10 15 20 25 30 35];
    ret_data = zeros(1, length(nqs));
    for i = 1:length(nqs)
        data = load(strcat(dir, 'out_s-1000-1_90_0_', num2str(nqs(i)), '.txt'));
        m_data = mean(data);
        ret_data(i) = m_data(7);
    end
endfunction

function graphPerfQuads()
    nqs = [1 5 10 15 20 25 30 35];
    data = loadDir('experiments/latest/PerfQuads/data/');
    figure;
    plot(nqs, data, 'b+-', 'linewidth', 4);
    ylim([min(data) - 5, max(data) + 5]);
endfunction

graphPerfQuads();
waitForEnd();
