function convertRRAtoStates(rraFile,rraModel,outputFile)

    %Inputs
    %
    % rraFile       .sto file containing RRA kinematics. These kinematics
    %               should only have the coordinate name (i.e. not the
    %               absolute path string to the coordinate)
    %
    % outputFile    optional filename to include for the output (will
    %               default to coordinates.sto)
    
    import org.opensim.modeling.*
    
    if nargin < 2
       error('At least 2 input of the RRA kinematics file and RRA model object is required.') 
    end
    if nargin < 3
        outputFile = 'coordinates.sto';
    end
    
    %Load in the RRA data
    RRAstorage = Storage(rraFile);

    %Create a copy of the RRA data to alter the column labels in
    statesStorage = Storage(rraFile);

    %Resample the data points linearly to avoid any later issues with matching
    %time points. Use a time stamp for 250 Hz
    RRAstorage.resampleLinear(1/250);
    statesStorage.resampleLinear(1/250);

    %Get the column headers for the storage file
    angleNames = RRAstorage.getColumnLabels();

    %Get the corresponding full paths from the RRA model to rename the
    %angles in the kinematics file
    for ii = 0:angleNames.getSize()-1
        currAngle = string(angleNames.get(ii));
        if ~strcmp(currAngle,'time')
            %Get full path to coordinate
            fullPath = [char(rraModel.getCoordinateSet().get(currAngle).getAbsolutePathString()),'/value'];
            %Set angle name appropriately using full path
            angleNames.set(ii,fullPath);
            %Cleanup
            clear currAngle fullPath
        end
    end
    clear ii

    %Set the states storage object to have the updated column labels
    statesStorage.setColumnLabels(angleNames);

    %Somewhere along the way the sates storage object changes to not be in
    %degrees, so set it back
    statesStorage.setInDegrees(true);

    %Write the states storage object to file
    statesStorage.print(outputFile);
    
end