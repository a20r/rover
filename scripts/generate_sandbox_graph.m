
function waitForEnd()
    for i = 1:10000
        k = waitforbuttonpress;
        if k == 1
            break;
        end
    end
endfunction

data = load('sandbox/all.txt');
plot(data(:, 1))
waitForEnd()
