function [hLine] = drawCircle(Center, Radius, StartDeg, EndDeg, LineStyle, LineColor, LineWidth)

    % prepare range circles
    Angle = linspace(StartDeg,EndDeg,100);
    CircleFun = @(ang)  [Radius*cosd(ang);  Radius*sind(ang)] + Center;
    Circle = CircleFun(Angle);
    
    % plot
    hLine = plot(Circle(1,:), Circle(2,:), 'LineStyle', LineStyle, 'Color', LineColor, 'LineWidth', LineWidth);
end