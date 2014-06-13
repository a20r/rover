
function [avg_ta, avg_sq, avg_risk] = getAverages(rn, qn)
    data = load(strcat("data/out_", num2str(rn), "_", num2str(qn), ".txt"));
    avg_ta = mean(data(:, 1));
    avg_sq = mean(data(:, 2));
    avg_risk = mean(data(:, 3));
endfunction

function [t_avg_ta, t_avg_sq, t_avg_risk] = getAverageMatrices(rns, qns)

    t_avg_ta = zeros(size(rns)(2), size(qns)(2));
    t_avg_sq = zeros(size(rns)(2), size(qns)(2));
    t_avg_risk = zeros(size(rns)(2), size(qns)(2));

    for ri = 1:size(rns)(2)
        for qi = 1:size(qns)(2)
            [avg_ta, avg_sq, avg_risk] = getAverages(rns(ri), qns(qi));
            t_avg_ta(ri, qi) = avg_ta;
            t_avg_sq(ri, qi) = avg_sq;
            t_avg_risk(ri, qi) = avg_risk;
        end
    end
endfunction

function plotAll(rns, qns)
    [QNS, RNS] = meshgrid(qns, rns);
    [t_avg_ta, t_avg_sq, t_avg_risk] = getAverageMatrices(rns, qns);
    figure;
    % hold on;

    subplot(2, 2, 1);
    surf(QNS, RNS, t_avg_ta);
    title("Total Area");
    xlabel("Number of Quads");
    ylabel("Number of Risk Points");

    subplot(2, 2, 2);
    surf(QNS, RNS, t_avg_sq);
    title("Sensor Quality");
    xlabel("Number of Quads");
    ylabel("Number of Risk Points");

    subplot(2, 2, 3);
    surf(QNS, RNS, t_avg_risk);
    xlabel("Number of Quads");
    ylabel("Number of Risk Points");
    title("Average Risk");

    subplot(2, 2, 4);
    plot(t_avg_ta(:, 1));
endfunction

function waitForEnd()
    for i = 1:10000
        k = waitforbuttonpress;
        if k == 1
            break;
        end
    end
endfunction

plotAll(21, 26);
waitForEnd();



