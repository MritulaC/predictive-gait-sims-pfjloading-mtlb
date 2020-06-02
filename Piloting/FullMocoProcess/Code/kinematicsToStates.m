function kinematicsToStates(kinematicsFile,osimModel,outputFile,inDegrees,outDegrees)

    %Inputs
    %
    % kinematicsFile        file containing kinematics. These kinematics
    %                       should only have the coordinate name (i.e. not the
    %                       absolute path string to the coordinate)
    %
    % osimModel             Opensim model object that corresponds to the
    %                       kinematics data
    %
    % outputFile            optional filename to include for the output (will
    %                       default to coordinates.sto)
    % inDegrees             Set to true if the kinematicsFile input is in degrees
    % outDegrees            Set to true if the desired output is in degrees.
    %                       Typically radians files have been working better with
    %                       Moco as there is no additional confusion with
    %                       conversion steps (see Moco GitHub issue #616)
    
    import org.opensim.modeling.*
    
    if nargin < 2
       error('At least 2 input of the kinematics file and model object is required.') 
    end
    if nargin < 3
        outputFile = 'coordinates.sto';
    end
    if nargin < 4
        inDegrees = false;
    end
    if nargin < 5
        outDegrees = false;
    end
    
    %Load in the RRA data
    kinematicsStorage = Storage(kinematicsFile);

    %Create a copy of the RRA data to alter the column labels in
    statesStorage = Storage(kinematicsFile);

    %Resample the data points linearly to avoid any later issues with matching
    %time points. Use a time stamp for 250 Hz
    kinematicsStorage.resampleLinear(1/250);
    statesStorage.resampleLinear(1/250);

    %Get the column headers for the storage file
    angleNames = kinematicsStorage.getColumnLabels();

    %Get the corresponding full paths from the RRA model to rename the
    %angles in the kinematics file
    for ii = 0:angleNames.getSize()-1
        currAngle = string(angleNames.get(ii));
        if ~strcmp(currAngle,'time')
            %Get full path to coordinate
            fullPath = [char(osimModel.getCoordinateSet().get(currAngle).getAbsolutePathString()),'/value'];
            %Set angle name appropriately using full path
            angleNames.set(ii,fullPath);
            %Cleanup
            clear currAngle fullPath
        end
    end
    clear ii

    %Set the states storage object to have the updated column labels
    statesStorage.setColumnLabels(angleNames);
    
    if inDegrees && ~outDegrees
        %Convert degrees values to radians for consistency with the current
        %file label (defaults back to inDegrees=no). Radians seem to work
        %better with the Moco process as well.
        osimModel.initSystem();
        osimModel.getSimbodyEngine().convertDegreesToRadians(statesStorage);
    elseif inDegrees && outDegrees
        %Change the storage label back to specifying indegrees=yes
        statesStorage.setInDegrees(true)
    elseif ~inDegrees && outDegrees
        %Convert radians to degrees
        osimModel.initSystem();
        osimModel.getSimbodyEngine().convertRadiansToDegrees(statesStorage);
        %Reset labeling for degrees
        statesStorage.setInDegrees(true)
    end

    %Write the states storage object to file
    statesStorage.print(outputFile);
    
end