clc; clear; format long;

plots = initializePlots();

linkage = Linkage(plots);
% linkage.analyzeStatics()
linkage.analyzeDynamics()

function plots = initializePlots()
    % Set up figures
    plots = struct('jointLinPos', {},...
                   'jointLinVelX', {},...
                   'jointLinVelY', {},...
                   'linkAngVel', {},...
                   'linkAngAccel', {}...
                  );
    % 'jointAccel', {},...
    % 'accel', {},...
    % 'staticForce', {},...
    % 'staticTorque', {},...
    % 'dynamicForce', {},...
    % 'dynamicTorque', {},...
    % plots = struct('jointPos', {},...
    %                'jointVel', {})
    plotNames = fieldnames(plots);
    plots = struct();

    for i = 1:numel(plotNames)
        fieldName = plotNames{i};
        figure('Name', fieldName);
        plots.(fieldName) = axes;
        hold(plots.(fieldName), 'on')
        if (fieldName ~= "linkAngAccel") | (fieldName ~= "linkAngVelX") | (fieldName ~= "linkAngVelY")
            axis(plots.(fieldName), 'equal')
        end
        if (fieldName == "linkAngVelX") | (fieldName == "linkAngVelY")
            axis(plots.(fieldName), 'tight')
        end
    end
end
