function [data json] = parse_json(json)
% [DATA JSON] = PARSE_JSON(json)
% This function parses a JSON string and returns a cell array with the
% parsed data. JSON objects are converted to structures and JSON arrays are
% converted to cell arrays.
%
% Example:
% google_search = 'http://ajax.googleapis.com/ajax/services/search/web?v=1.0&q=matlab';
% matlab_results = parse_json(urlread(google_search));
% disp(matlab_results{1}.responseData.results{1}.titleNoFormatting)
% disp(matlab_results{1}.responseData.results{1}.visibleUrl)

    data = cell(0,1);

    while ~isempty(json)
        [value json] = parse_value(json);
        data{end+1} = value; %#ok<AGROW>
    end
end

function [value json] = parse_value(json)
    value = [];
    if ~isempty(json)
        id = json(1);
        json(1) = [];

        json = strtrim(json);

        switch lower(id)
            case '"'
                [value json] = parse_string(json);

            case '{'
                [value json] = parse_object(json);

            case '['
                [value json] = parse_array(json);

            case 't'
                value = true;
                if (length(json) >= 3)
                    json(1:3) = [];
                else
                    ME = MException('json:parse_value',['Invalid TRUE identifier: ' id json]);
                    ME.throw;
                end

            case 'f'
                value = false;
                if (length(json) >= 4)
                    json(1:4) = [];
                else
                    ME = MException('json:parse_value',['Invalid FALSE identifier: ' id json]);
                    ME.throw;
                end

            case 'n'
                value = [];
                if (length(json) >= 3)
                    json(1:3) = [];
                else
                    ME = MException('json:parse_value',['Invalid NULL identifier: ' id json]);
                    ME.throw;
                end

            otherwise
                [value json] = parse_number([id json]); % Need to put the id back on the string
        end
    end
end

function [data json] = parse_array(json)
    data = cell(0,1);
    while ~isempty(json)
        if strcmp(json(1),']') % Check if the array is closed
            json(1) = [];
            return
        end

        [value json] = parse_value(json);

        if isempty(value)
            ME = MException('json:parse_array',['Parsed an empty value: ' json]);
            ME.throw;
        end
        data{end+1} = value; %#ok<AGROW>

        while ~isempty(json) && ~isempty(regexp(json(1),'[\s,]','once'))
            json(1) = [];
        end
    end
end

function [data json] = parse_object(json)
    data = [];
    while ~isempty(json)
        id = json(1);
        json(1) = [];

        switch id
            case '"' % Start a name/value pair
                [name value remaining_json] = parse_name_value(json);
                if isempty(name)
                    ME = MException('json:parse_object',['Can not have an empty name: ' json]);
                    ME.throw;
                end
                data.(name) = value;
                json = remaining_json;

            case '}' % End of object, so exit the function
                return

            otherwise % Ignore other characters
        end
    end
end

function [name value json] = parse_name_value(json)
    name = [];
    value = [];
    if ~isempty(json)
        [name json] = parse_string(json);

        % Skip spaces and the : separator
        while ~isempty(json) && ~isempty(regexp(json(1),'[\s:]','once'))
            json(1) = [];
        end
        [value json] = parse_value(json);
    end
end

function [string json] = parse_string(json)
    string = [];
    while ~isempty(json)
        letter = json(1);
        json(1) = [];

        switch lower(letter)
            case '\' % Deal with escaped characters
                if ~isempty(json)
                    code = json(1);
                    json(1) = [];
                    switch lower(code)
                        case '"'
                            new_char = '"';
                        case '\'
                            new_char = '\';
                        case '/'
                            new_char = '/';
                        case {'b' 'f' 'n' 'r' 't'}
                            new_char = sprintf('\%c',code);
                        case 'u'
                            if length(json) >= 4
                                new_char = sprintf('\\u%s',json(1:4));
                                json(1:4) = [];
                            end
                        otherwise
                            new_char = [];
                    end
                end

            case '"' % Done with the string
                return

            otherwise
                new_char = letter;
        end
        % Append the new character
        string = [string new_char]; %#ok<AGROW>
    end
end

function [num json] = parse_number(json)
    num = [];
    if ~isempty(json)
        % Validate the floating point number using a regular expression
        [s e] = regexp(json,'^[\w]?[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?[\w]?','once');
        if ~isempty(s)
            num_str = json(s:e);
            json(s:e) = [];
            num = str2double(strtrim(num_str));
        end
    end
end
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
    hold on;
    title("Total Area");
    xlabel("Number of Quads");
    ylabel("Number of Risk Points");
    surf(QNS, RNS, t_avg_ta);
    figure;
    hold on;
    title("Sensor Quality");
    xlabel("Number of Quads");
    ylabel("Number of Risk Points");
    surf(QNS, RNS, t_avg_sq);
    figure;
    hold on;
    xlabel("Number of Quads");
    ylabel("Number of Risk Points");
    title("Average Risk");
    surf(QNS, RNS, t_avg_risk);
endfunction

function waitForEnd()
    for i = 1:10000
        k = waitforbuttonpress;
        if k == 1
            break;
        end
    end
endfunction

function main()
    st = fileread('config.json');
    config_json = parse_json(fileread('config.json')){1};
    rns = config_json.min_risk_points:config_json.risk_points_step:config_json.max_risk_points + config_json.risk_points_step;
    qns = config_json.min_quads:config_json.quads_step:config_json.max_quads + config_json.quads_step;

    plotAll(rns, qns);
    waitForEnd();
endfunction

main()
