f=fopen('chip.txt');
outline = fscanf(f, '%d %d %d %d', [1 4]);
rectangle('Position', outline, 'FaceColor', "w", 'EdgeColor', 'k', 'LineWidth', 2)

axis image;

numBlock = fscanf(f, '%d', [1 1]);
for i = 1 : numBlock   
    name = fscanf(f, '%s', [1 1]);
    %rect = fscanf(f, '%f %f %f %f ', [1 4]);
    numPoint = fscanf(f, '%d', [1 1]);
    x = zeros(1, numPoint);
    y = zeros(1, numPoint);
    for j = 1 : numPoint
        x(j) = fscanf(f, '%d', [1 1]);
        y(j) = fscanf(f, '%d', [1 1]);
    end
    feedthroughable = fscanf(f, '%d', [1 1]);
    if feedthroughable == 1
        %color = "#FFA042";
        % color = 'c';
        color = [1 0.627 0.259];
    else
        %color = "#ADADAD";
        % color = 'r';
        color = [0.678 0.678 0.678];
    end
    patch(x, y, color, 'EdgeColor', 'k', 'LineWidth', 1);
    % if name == "B8"
    if 0
        text((max(x)+min(x))/2, (max(y)+min(y))/2, {name}, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end
    clear x;
    clear y;
end

numRegion = fscanf(f, '%d', [1 1]);
for i = 1 : numRegion   
    name = fscanf(f, '%s', [1 1]);
    rect = fscanf(f, '%f %f %f %f ', [1 4]);
    rectangle('Position', rect, 'FaceColor', "#a1cbf5", 'EdgeColor', 'k', 'LineWidth', 1);
    % if name == "R12"
    if 0
        text(rect(1)+rect(3)/2, rect(2)+rect(4)/2, name, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end
end

numNet = fscanf(f, '%d', [1 1]);
colors = {'c', 'r', 'b', 'g', 'y', [.5 .6 .7], [.8 .2 .6]};
for i = 1 : numNet
    if i <= 5
        color = colors{i};
    else
        color = rand(1, 3);
    end

    numPin = fscanf(f, '%d', [1 1]);
    for j = 1 : numPin
        x = fscanf(f, '%d', [1 1]);
        y = fscanf(f, '%d', [1 1]);
        hold on;
        plot(x, y, '.', 'MarkerSize', 20, 'Color', color);
    end

    numTwoPin = fscanf(f, '%d', [1 1]);
    for j = 1 : numTwoPin
        numSegment = fscanf(f, '%d', [1 1]);
        for k = 1 : numSegment
            x1 = fscanf(f, '%d', [1 1]);
            y1 = fscanf(f, '%d', [1 1]);
            x2 = fscanf(f, '%d', [1 1]);
            y2 = fscanf(f, '%d', [1 1]);
            plot([x1 x2], [y1 y2], 'Color', color, 'LineWidth', 1);
        end
    end
end



numSoft = fscanf(f, '%d', [1 1]);
for i = 1 : numSoft
    name = fscanf(f, '%s', [1 1]);
    numPoint = fscanf(f, '%d', [1 1]);
    for j = 1 : numPoint
        x(j) = fscanf(f, '%d', [1 1]);
        y(j) = fscanf(f, '%d', [1 1]);
    end
    patch(x, y, 'c', 'EdgeColor', 'k', 'LineWidth', 1);
 %  text((max(x)+min(x))/2, (max(y)+min(y))/2, {name, polyarea(x, y), polyarea(x, y)/((max(x)-min(x))*(max(y)-min(y))), (max(x)-min(x))/(max(y)-min(y))}, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
 %  text((max(x)+min(x))/2, (max(y)+min(y))/2, {name, (max(x)+min(x))/2, (max(y)+min(y))/2}, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    text((max(x)+min(x))/2, (max(y)+min(y))/2, {name}, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    clear x;
    clear y;
end