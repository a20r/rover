
function makeGraphs()
    figure;
    risk = load('scenes/s_1000_1.out');
    risk = risk';
    pcolor(risk);
    hold on;
    xlabel('X'); ylabel('Y');
    shading interp;
    xlim([0 1000]); ylim([0 1000]);

    figure;
    risk = load('scenes/s_1000_2.out');
    risk = risk';
    pcolor(risk);
    hold on;
    xlabel('X'); ylabel('Y');
    shading interp;
    xlim([0 1000]); ylim([0 1000]);

    figure;
    risk = load('scenes/s_1000_3.out');
    risk = risk';
    pcolor(risk);
    hold on;
    xlabel('X'); ylabel('Y');
    shading interp;
    xlim([0 1000]); ylim([0 1000]);

    figure;
    risk = load('scenes/s_1500_1.out');
    risk = risk';
    pcolor(risk);
    hold on;
    xlabel('X'); ylabel('Y');
    shading interp;
    xlim([0 1500]); ylim([0 1500]);

    figure;
    risk = load('scenes/s_1500_2.out');
    risk = risk';
    pcolor(risk);
    hold on;
    xlabel('X'); ylabel('Y');
    shading interp;
    xlim([0 1500]); ylim([0 1500]);

    figure;
    risk = load('scenes/s_1500_3.out');
    risk = risk';
    pcolor(risk);
    hold on;
    xlabel('X'); ylabel('Y');
    shading interp;
    xlim([0 1500]); ylim([0 1500]);

    figure;
    risk = load('scenes/s_2000_1.out');
    risk = risk';
    pcolor(risk);
    hold on;
    xlabel('X'); ylabel('Y');
    shading interp;
    xlim([0 2000]); ylim([0 2000]);

    figure;
    risk = load('scenes/s_2000_2.out');
    risk = risk';
    pcolor(risk);
    hold on;
    xlabel('X'); ylabel('Y');
    shading interp;
    xlim([0 2000]); ylim([0 2000]);

    figure;
    risk = load('scenes/s_2000_3.out');
    risk = risk';
    pcolor(risk);
    hold on;
    xlabel('X'); ylabel('Y');
    shading interp;
    xlim([0 2000]); ylim([0 2000]);
endfunction

function waitForEnd()
    for i = 1:10000
        k = waitforbuttonpress;
        if k == 1
            break;
        end
    end
endfunction

makeGraphs();
waitForEnd();


