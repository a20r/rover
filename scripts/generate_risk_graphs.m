
function makeGraphs(scene, dim)
    % f = figure('visible','off');
    risk = load(scene);
    r_dim = size(risk)(1);
    [X, Y] = meshgrid(0:10:dim, 0:10:dim);
    pcolor(X, Y, risk((r_dim - dim):10:r_dim, (r_dim - dim):10:r_dim));
    hold on;
    xlabel('x position [m]');
    ylabel('y position [m]');
    shading interp;
    xlim([0 dim]); ylim([0 dim]);
    fontname = 'Helvetica';
    fontsize = 20;
    set([gca; findall(gca, 'Type','text')], 'FontSize', fontsize);
    set([gca; findall(gca, 'Type','text')], 'FontName', fontname);
    print(strcat('sandbox/figs_tase/', scene, '.png'), '-dpng');
    hold off;
endfunction

function waitForEnd()
    for i = 1:10000
        k = waitforbuttonpress;
        if k == 1
            break;
        end
    end
endfunction

% makeGraphs('scenes/s-300-0.out', 300);
% makeGraphs('scenes/s-300-1.out', 300);
% makeGraphs('scenes/s-300-2.out', 300);
% makeGraphs('scenes/s-500-0.out', 500);
% makeGraphs('scenes/s-500-1.out', 500);
% makeGraphs('scenes/s-500-2.out', 500);
makeGraphs('scenes/s-1000-0.out', 1000);
makeGraphs('scenes/s-1000-1.out', 1000);
makeGraphs('scenes/s-1000-2.out', 1000);
% waitForEnd();
