function [] = prettyPlots()
%PRETTYPLOTS improves plot visualization for presentation

    %% Get all the line handles from root and set its width
    allLineHandles = findall(groot, 'Type', 'line');
    set(allLineHandles, 'LineWidth', 2.0);

    %% Get all axes handles and set its color
    allAxesHandles = findall(groot, 'Type', 'Axes');
    set(allAxesHandles, 'FontName','Arial','FontWeight','Bold','LineWidth',3,...
        'FontSize',12);

    %% Get titles
    alltext = findall(groot, 'Type', 'Text');
    set(alltext,'FontName','Arial','FontWeight','Bold','FontSize',12, 'Interpreter', 'None');
end

