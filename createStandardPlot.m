function createStandardPlot(slice, theta)
    figure(slice); hold on; box on; axis equal; legend('show'); 
    set(groot, 'defaultTextInterpreter', 'latex');
    set(groot, 'defaultLegendInterpreter', 'latex');
    set(gcf, "Color", "w")
    title("slice = " + string(slice) +" where " + "$\theta =$ " + string(rad2deg(theta)) + " $[deg]$")
end

