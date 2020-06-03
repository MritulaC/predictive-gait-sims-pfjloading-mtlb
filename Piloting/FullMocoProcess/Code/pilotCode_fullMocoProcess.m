%% This function serves as pilot processing for running experimental data
%  from its raw form (i.e. .trc and .mot) through all Moco processes to
%  tracking and predictive simulations, along the way trying to make use of
%  relevant Moco features to produce high quality predictive simulations.

import org.opensim.modeling.*
warning off

%Set main directory
mainDir = pwd;
addpath(genpath(mainDir));

%Add model geometry directory
ModelVisualizer.addDirToGeometrySearchPaths('..\GenericModel\Geometry');

%% Model scaling

%Set up a scale tool
scaleTool = ScaleTool();

%Get participant mass from details file
d = importdata('..\Data\DT_participantDetails.txt');
height = d.data(1); mass = d.data(2);
clear d

%Set mass in scale tool
scaleTool.setSubjectMass(mass);

%Navigate to generic model directory
cd('..\GenericModel'); genModelDir = [pwd,'\'];

%Set generic model file
scaleTool.getGenericModelMaker().setModelFileName([pwd,'\pfjLoading_GenericOsimModel_LaiArnold.osim']);

%Set the measurement set
measurementSetObject = OpenSimObject.makeObjectFromFile([pwd,'\scaleMeasurementSet.xml']);
measurementSet = MeasurementSet.safeDownCast(measurementSetObject);
scaleTool.getModelScaler().setMeasurementSet(measurementSet);

%Set scale tasks
taskSet = IKTaskSet([pwd,'\scaleTasks.xml']);
for k = 0:taskSet.getSize()-1
    scaleTool.getMarkerPlacer().getIKTaskSet().adoptAndAppend(taskSet.get(k));
end
clear k

%Set marker file
cd('..\Data\');
%Add the virtual markers and create a new .trc file to use in scaling
addVirtualMarkersStatic('static.trc','staticVirtualMarkers.trc');
%Place in scale tool
scaleTool.getMarkerPlacer().setMarkerFileName([pwd,'\staticVirtualMarkers.trc']);
scaleTool.getModelScaler().setMarkerFileName([pwd,'\staticVirtualMarkers.trc']);

%Set options
scaleTool.getModelScaler().setPreserveMassDist(true);
scaleOrder = ArrayStr(); scaleOrder.set(0,'measurements');
scaleTool.getModelScaler().setScalingOrder(scaleOrder);

%Set time ranges
timeRange = ArrayDouble();
timeRange.set(0,0.5); timeRange.set(1,1.5);
scaleTool.getMarkerPlacer().setTimeRange(timeRange);
scaleTool.getModelScaler().setTimeRange(timeRange);

%Set output files
scaleTool.getModelScaler().setOutputModelFileName([pwd,'\scaledModel.osim']);
scaleTool.getModelScaler().setOutputScaleFileName([pwd,'\scaleSet.xml']);

%Set marker adjuster parameters
scaleTool.getMarkerPlacer().setOutputMotionFileName([pwd,'\static_motion.mot']);
scaleTool.getMarkerPlacer().setOutputModelFileName([pwd,'\scaledModelAdjusted.osim']);

%Save and run scale tool
scaleTool.print([pwd,'\scaleSetup.xml']);
scaleTool.run();

%Load in scaled model
scaledModel = Model('scaledModelAdjusted.osim');

%The locations and size of the contact sphere parameters go unchanged with
%standard model scaling, so these need to be edited to ensure they are in
%an appropriate place. This can be done based on the scale set parameters
%for the representative bodies.

%Load in the scale set
[scaleSetTree, ~, ~] = xml_read('scaleSet.xml');

%Loop through and identify the indices of the relevant segment bodies to
%get data from
for bb = 1:length(scaleSetTree.ScaleSet.objects.Scale)
    if strcmp(scaleSetTree.ScaleSet.objects.Scale(bb).segment,'calcn_r')
        heelBodyInd_r = bb;
    elseif strcmp(scaleSetTree.ScaleSet.objects.Scale(bb).segment,'calcn_l')
        heelBodyInd_l = bb;
    elseif strcmp(scaleSetTree.ScaleSet.objects.Scale(bb).segment,'toes_r')
        toesBodyInd_r = bb;
    elseif strcmp(scaleSetTree.ScaleSet.objects.Scale(bb).segment,'toes_l')
        toesBodyInd_l = bb;
    end
end
clear bb

%Get the 3D scale factors for the relevant bodies and average to scale the
%sphere radii. Note that these scale factors for each foot will be the same
%for heel and toes as it looks like same scale factors are applied.
heelSphereScale_r = sum(scaleSetTree.ScaleSet.objects.Scale(heelBodyInd_r).scales) / 3;
heelSphereScale_l = sum(scaleSetTree.ScaleSet.objects.Scale(heelBodyInd_l).scales) / 3;
toesSphereScale_r = sum(scaleSetTree.ScaleSet.objects.Scale(toesBodyInd_r).scales) / 3;
toesSphereScale_l = sum(scaleSetTree.ScaleSet.objects.Scale(toesBodyInd_l).scales) / 3;

%Scale the radii to each of their respective factors
%While accessing the spheres, also adjust their position based on the scale
%factor for the respective axes
%Create a list of the sheres to loop through and edit
sphereList = [{'heel_r'}; {'mh1_r'}; {'mh3_r'}; {'mh5_r'}; {'hallux_r'}; {'othertoes_r'};
    {'heel_l'}; {'mh1_l'}; {'mh3_l'}; {'mh5_l'}; {'hallux_l'}; {'othertoes_l'}];
%Loop through sphere list
for ss = 1:length(sphereList)
    %Get sphere
    currSphere = ContactSphere.safeDownCast(scaledModel.getContactGeometrySet().get(sphereList{ss}));
    %Set the current scaling factor & body to get based on sphere name
    if contains(sphereList{ss},'_r')
        if contains(sphereList{ss},'hallux') || contains(sphereList{ss},'othertoes')
            scaleFac = toesSphereScale_r;
            getBod = toesBodyInd_r;
        else
            scaleFac = heelSphereScale_r;
            getBod = heelBodyInd_r;
        end
    elseif contains(sphereList{ss},'_l')
        if contains(sphereList{ss},'hallux') || contains(sphereList{ss},'othertoes')
            scaleFac = toesSphereScale_l;
            getBod = toesBodyInd_l;
        else
            scaleFac = heelSphereScale_l;
            getBod = heelBodyInd_l;
        end
    end
    %Rescale the radius
    currSphere.setRadius(currSphere.getRadius * scaleFac);
    newLoc(1) = scaleSetTree.ScaleSet.objects.Scale(getBod).scales(1) * currSphere.getLocation().get(0);
    newLoc(2) = scaleSetTree.ScaleSet.objects.Scale(getBod).scales(2) * currSphere.getLocation().get(1);
    newLoc(3) = scaleSetTree.ScaleSet.objects.Scale(getBod).scales(3) * currSphere.getLocation().get(2);
    currSphere.setLocation(Vec3(newLoc(1),newLoc(2),newLoc(3)));    
end
clear ss

%Reset model name
scaledModel.setName('scaledModel');

%Finalise connections and update scaled model
scaledModel.finalizeConnections();
scaledModel.print('scaledModelAdjusted.osim');

%Scale muscle strength based on linear function presented in Handsfield
%et al. (2014). This uses some convenience functions that are packaged
%with the Rajagopal et al. (2016) gait model. Note that the height of
%the generic model is 1.700
genModel = Model([genModelDir,'pfjLoading_GenericOsimModel_LaiArnold.osim']);
scaledModelMuscle = scaleOptimalForceSubjectSpecific(genModel,scaledModel,1.700,height);
scaledModelMuscle.print('scaledModelMuscle.osim');

%% Identify the time parameters for the dynamic simulations

%Load the GRF data
grfTable = TimeSeriesTable('Jog05_grf.mot');
grfData = osimTableToStruct(grfTable);

%Create a logical for where the right & left foot is in contact with the ground
%based on 50N threshold (which is what was originally used)
rightGrfOn = logical(grfData.calcn_r_ground_force_vy > 50);
leftGrfOn = logical(grfData.calcn_l_ground_force_vy > 50);

%Identify the index where right and left GRF starts
rightGrfDiff = diff(rightGrfOn);
rightGrfOnInd = find(rightGrfDiff == 1) + 1;
leftGrfDiff = diff(leftGrfOn);
leftGrfOnInd = find(leftGrfDiff == 1) + 1;

%Find the middle right foot strike
rightStartInd = rightGrfOnInd(round(length(rightGrfOnInd)/2));

%Find the next left foot strike to signify the half gait cycle
leftStartInd = leftGrfOnInd(find(leftGrfOnInd > rightStartInd,1));

%Identify the times corresponding to these foot strikes
startTime = grfData.time(rightStartInd);
endTime = grfData.time(leftStartInd);

%Set number of mesh intervals at 50 for a half gait cycle as per Fallise et
%al. study
nMeshIntervals = 50;

%% Run a marker tracking simulation through Moco

%Create a copy of the model to use in the tracking simulation that we can
%clear the force set and replace with torque driven actuators
markerTrackModel = Model('scaledModelMuscle.osim');

%Clear this models forceset
markerTrackModel.updForceSet().clearAndDestroy();

%Clear contact geometry set now that the spheres are gone
markerTrackModel.updContactGeometrySet().clearAndDestroy();

%Loop through and add coordinate actuators back into the models forceset to
%drive the motion.
for cc = 0:markerTrackModel.getCoordinateSet().getSize()-1
    
    %Create the coordinate actuator
    actu = CoordinateActuator();
    
    %Get name of the current coordinate
    currCoord = markerTrackModel.getCoordinateSet().get(cc).getName();
    
    %Set actuator coordinate
    actu.setCoordinate(markerTrackModel.getCoordinateSet().get(cc));
    
    %Set actuator name
    actu.setName(['tau_',char(currCoord)]);
    
    %Set optimal force
    actu.setOptimalForce(1000);
    
    %Set control levels    
    actu.setMaxControl(inf);
    actu.setMinControl(-inf);
    
    %Add actuator to model
    markerTrackModel.updForceSet().cloneAndAppend(actu);
    
end
clear cc

%Finalise model connections
markerTrackModel.finalizeConnections();

%Create a MocoStudy
markerTrackStudy = MocoStudy();
markerTrackStudy.setName('markerTracking');

%Define the problem
markerTrackProblem = markerTrackStudy.updProblem();

%Set model
markerTrackProblem.setModel(markerTrackModel);

%Set time bounds in the problem
markerTrackProblem.setTimeBounds(startTime,endTime);

%Create a marker tracking cost term
markerTrackingCost = MocoMarkerTrackingGoal();
markerTrackingCost.setName('marker_tracking');

%Load up the weight on the marker tracking cost to make it have a much
%higher importance
markerTrackingCost.setWeight(1e5);

%Create a set of marker weights used to define the relative importance of
%tracking individual markers. For now we'll use the same weights specified
%in the ik tasks file used in the original data processing.

%Set the marker weights object
markerWeights = SetMarkerWeights();

%Load in the IK task set
ikTaskSet = IKTaskSet('..\GenericModel\ikTasks.xml');

%Loop through IK tasks and add to the marker weight set
for ii = 0:ikTaskSet.getSize()-1    
    %Check to see whether to apply this marker in the analysis
    if ikTaskSet.get(ii).getApply()    
        %Add current marker with appropriate weight to the weight set
        markerWeights.cloneAndAppend(MarkerWeight(char(ikTaskSet.get(ii).getName()),...
            ikTaskSet.get(ii).getWeight()));    
    end   
end
clear ii

%Create a markers reference from the experimental marker trajectories
markersRef = MarkersReference();
markersRef.set_marker_file([pwd,'\Jog05.trc']);

%Set the marker weights in the reference
markersRef.setMarkerWeightSet(markerWeights);

%Add the reference to the tracking cost
markerTrackingCost.setMarkersReference(markersRef);

%Allow the option to ignore unused markers in the tracking cost
markerTrackingCost.setAllowUnusedReferences(true);

%Add the marker tracking goal to the problem
markerTrackProblem.addGoal(markerTrackingCost);

%Add a low-weighted control effort cost to reduce oscillations in the actuator controls.
controlCost = MocoControlGoal();
controlCost.setWeight(0.001);
markerTrackProblem.addGoal(controlCost);

%%% frame distance constraint...add this as a comparator option...

%Configure the solver
markerTrackSolver = markerTrackStudy.initCasADiSolver();
markerTrackSolver.set_num_mesh_intervals(nMeshIntervals);
markerTrackSolver.set_optim_constraint_tolerance(1e-3);
markerTrackSolver.set_optim_convergence_tolerance(1e-3);

%Set a default guess in the solver
markerTrackSolver.setGuess('bounds');

%Print study for viewing
markerTrackStudy.print('markerTracking.omoco');

%Run the solver
clc
markerTrackSolution = markerTrackStudy.solve();

%%%%% Marker tracking doesn't work that great, the motion doesn't look
%%%%% right. We can do a comparison with an inverse kinematics process
%%%%% using the same weights over the same time period.

%%%%% Could set coordinates from IK as initial guess to potentially get the
%%%%% marker tracking to work better?

%% Run inverse kinematics analysis

%%%%%% TODO: could add virtual markers to the IK problem
%%%%%% Better scaling does however better align feet with ground

%Initialise IK tool
ikTool = InverseKinematicsTool();

%Set model
ikTool.setModel(markerTrackModel);

%Set task set
ikTool.set_IKTaskSet(ikTaskSet);

%Set marker file
ikTool.set_marker_file([pwd,'\Jog05.trc']);

%Set times
ikTool.setStartTime(startTime);
ikTool.setEndTime(endTime);

%Set output filename
ikTool.set_output_motion_file('ikResults.mot');

%Run IK
clc
ikTool.run();

%% Compare IK vs. marker tracking

%Import marker tracking data
markerTrackData = importdata('markerTracking_solution.sto');

%Extract time
markerTrackResults.time = markerTrackData.data(:,1);

%Extract kinematic data
for cc = 1:length(markerTrackData.colheaders)
    currState = markerTrackData.colheaders{cc};
    %Check for kinematic data
    if contains(currState,'/jointset') && contains(currState,'/value')
        %Identify joint coordinate name
        splitStr = strsplit(currState,'/');
        currCoord = splitStr{4};
        %Extract data
        markerTrackResults.kinematics.(char(currCoord)) = markerTrackData.data(:,cc);
        %Cleanup
        clear currCoord splitStr
    end
end
clear cc

%Import IK data
ikData = importdata('ikResults.mot');

%Extract time
ikResults.time = ikData.data(:,1);

%Extract kinematic data
for cc = 2:length(ikData.colheaders)
    %Extract data
    ikResults.kinematics.(ikData.colheaders{cc}) = ikData.data(:,cc);
end
clear cc

%Create plots to compare
coordPlot = [{'lumbar_extension'}; {'lumbar_bending'}; {'lumbar_rotation'};
    {'pelvis_tilt'}; {'pelvis_rotation'}; {'pelvis_list'};
    {'hip_flexion_r'}; {'hip_adduction_r'}; {'hip_rotation_r'};
    {'knee_angle_r'}; {'ankle_angle_r'}];
coordPlotName = [{'Lumar Extension'}; {'Lumbar Bending'}; {'Lumbar Rotation'};
    {'Pelvis Tilt'}; {'Pelvis Rotation'}; {'Pelvis List'};
    {'R. Hip Flexion'}; {'R. Hip Adduction'}; {'R. Hip Rotation'};
    {'R. Knee Angle'}; {'R. Ankle Angle'}];

%Plot
figure; hold on
for pp = 1:length(coordPlot)
    subplot(4,3,pp); hold on
    %IK result
    plot(ikResults.time,ikResults.kinematics.(coordPlot{pp}),'k','LineWidth',1.5);
    %Marker tracking result
    plot(markerTrackResults.time,rad2deg(markerTrackResults.kinematics.(coordPlot{pp})),'r','LineWidth',1.5);
    %Set title
    title(coordPlotName{pp});
    %Set x-limits
    ax = gca; ax.XLim = [ikResults.time(1) ikResults.time(end)]; clear ax
end
clear pp

%%%%% Plot results honestly look like the marker tracking is trying to
%%%%% minimise model movement (and subsequently the actuator controls
%%%%% effort) instead of appropriately tracking markers. May need to
%%%%% consider bumping up the overall marker tracking goal weight?

%%%%% Adding weight to the marker tracking was starting to get a little
%%%%% better, but still not great and was taking much longer to converge.
%%%%% Certain individual markers likely need to be weighted heavier (like
%%%%% the ones on the legs) to get a better result.

%% Run inverse tracking on IK

%Set up inverse tool
inverse = MocoInverse();

%Construct the MocoInverse tool.
inverse = MocoInverse();
inverse.setName('gaitInverse');

%Get the scaled muscle model to use in the inverse simulations
inverseModel = Model('scaledModelMuscle.osim');

%Clear this models forceset
inverseModel.updForceSet().clearAndDestroy();

%Clear contact geometry set now that the spheres are gone
inverseModel.updContactGeometrySet().clearAndDestroy();

%Loop through and add coordinate actuators back into the models forceset to
%drive the motion.
for cc = 0:inverseModel.getCoordinateSet().getSize()-1
    
    %Create the coordinate actuator
    actu = CoordinateActuator();
    
    %Get name of the current coordinate
    currCoord = inverseModel.getCoordinateSet().get(cc).getName();
    
    %Set actuator coordinate
    actu.setCoordinate(inverseModel.getCoordinateSet().get(cc));
    
    %Set actuator name
    actu.setName(['tau_',char(currCoord)]);
    
    %Set optimal force
    actu.setOptimalForce(1000);
    
    %Set control levels    
    actu.setMaxControl(inf);
    actu.setMinControl(-inf);
    
    %Add actuator to model
    inverseModel.updForceSet().cloneAndAppend(actu);
    
end
clear cc

%Finalise model connections
inverseModel.finalizeConnections();

%Set model in inverse problem
inverse_modelProcessor = ModelProcessor(inverseModel);
inverse_modelProcessor.append(ModOpAddExternalLoads([pwd,'\Jog05_grf.xml']));
inverse.set_model(inverse_modelProcessor);

%Convert IK file to states
kinematicsToStates('ikResults.mot',markerTrackModel,'ikResults_states.sto',true,false); 

%Construct a TableProcessor of the coordinate data and pass it to the
%inverse tool
inverse.setKinematics(TableProcessor([pwd,'\ikResults_states.sto']));

%Allow extra columns in kinemtics file just in case
inverse.set_kinematics_allow_extra_columns(true);

%Set timing details
%Times need to be taken from the IK file as they seem to be rounded
%compared to the GRF data (i.e. different marker sampling rate)
inverse.set_initial_time(Storage('ikResults.mot').getFirstTime());
inverse.set_final_time(Storage('ikResults.mot').getLastTime());
inverse.set_mesh_interval(0.01);

%Save problem to text to review if necessary
inverse.print('gaitInverse_fromIK.omoco'); 

%Solve problem
clc
inverseSolution_fromIK = inverse.solve();

%Rename the moco solution that comes from the inverse problem
movefile('MocoStudy_solution.sto','gaitInverse_fromIK.sto');

%Extract the inveser solution as a moco solution to grab all aspects of the
%solution not usually output (e.g. kinematics)
inverseSolution_fromIK.getMocoSolution().write('gaitInverse_fromIK.sto');

%Set as accesible object
inverseSolution_fromIK = inverseSolution_fromIK.getMocoSolution();

%Compare kinematics from inverse solution to IK

%Import inverse tracking data
inverseData_fromIK = importdata('gaitInverse_fromIK.sto');

%Extract time
inverseResults_fromIK.time = inverseData_fromIK.data(:,1);

%Extract kinematic data
for cc = 1:length(inverseData_fromIK.colheaders)
    currState = inverseData_fromIK.colheaders{cc};
    %Check for kinematic data
    if contains(currState,'/jointset') && contains(currState,'/value')
        %Identify joint coordinate name
        splitStr = strsplit(currState,'/');
        currCoord = splitStr{4};
        %Extract data
        inverseResults_fromIK.kinematics.(char(currCoord)) = inverseData_fromIK.data(:,cc);
        %Cleanup
        clear currCoord splitStr
    end
end
clear cc

%Plot
figure; hold on
for pp = 1:length(coordPlot)
    subplot(4,3,pp); hold on
    %IK result
    plot(ikResults.time,ikResults.kinematics.(coordPlot{pp}),'k','LineWidth',1.5);
    %Inverse result
    plot(inverseResults_fromIK.time,rad2deg(inverseResults_fromIK.kinematics.(coordPlot{pp})),'r--','LineWidth',1.5);
    %Set title
    title(coordPlotName{pp});
    %Set x-limits
    ax = gca; ax.XLim = [ikResults.time(1) ikResults.time(end)]; clear ax
end
clear pp

%%%%% Moco inverse results match those from IK exactly, signifying that the
%%%%% actuator controls can work to match the experimental kinematics and
%%%%% GRF data.

%%%%% Different scaling does, however, change this...

%% Run a states tracking simulation from the IK data

%Next step looks to do something similar to the above inverse tracking
%simulation, by specifying actuator controls to track rather than match the
%kinematic states and GRF data. Building from this would be to add on the
%GRF tracking as a prediction rather than using it as experimental data. we
%could also limit the contriubtions of the pelvis actuators in the cost
%function here to perform a similar function to RRA in altering the
%kinematics slightly to minimise the use of these actuators.

%This is a relatively simple problem compared to some existing attempts, as
%it doesn't have periodic goals or speed tacking goals etc. - it simply
%fouses on matching the states and minimising the controls.

%Set the global tracking weights for each aspect of the problem
controlEffortWeight = 0.001; %default
stateTrackingWeight = 5;

%Define the motion tracking problem
track = MocoTrack();
track.setName('gaitTracking_fromIK');

%Set the kinematics
track.setStatesReference(TableProcessor([pwd,'\ikResults_states.sto']));
track.set_states_global_tracking_weight(1.0);
track.set_allow_unused_references(true); 

%Set the model
%Can just re-use the same processor from earlier given that nothing has
%changed with the actuators and kinematic/GRF data
track.setModel(inverse_modelProcessor);

%Initialise the tracking object to recieve a pre-configured MocoStudy
%object based on the current track settings above. Use this to customise
%the problem beyond the track interface.
study = track.initialize();

% Get a reference to the study problem
problem = study.updProblem();

%Set state weights within problem
%Generic weights for everything right now
statesWeights = MocoWeightSet();
%Get state variable names
inverseModel_state = inverseModel.initSystem();
stateVars = inverseModel.getStateVariableNames();
%Convert to string array while only taking jointset states
r = 1;
for ss = 0:stateVars.getSize()-1
    currState = char(stateVars.get(ss));
    if contains(currState,'/jointset')
        jointStates{r,1} = currState;
        r = r + 1;
    end
end
clear ss
%Append each state weight to goal
%This sets all non pelvis weights to a value of 5 and the pelvis values to
%1, as a basic means to allow these to be altered to minimise residual
%pelvis use. This is a simplistic approach but a start to see how it works.
for ss = 1:length(jointStates)
    if contains(jointStates{ss},'pelvis')
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},1.0));
    else
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},5.0));
    end
end
clear ss
%Set states goal in study
statesGoal = MocoStateTrackingGoal.safeDownCast(problem.updGoal('state_tracking'));
statesGoal.setWeightSet(statesWeights);
statesGoal.setWeight(stateTrackingWeight);

%Get a reference to the MocoControlGoal that is added to every MocoTrack
%problem by default and change the weight
effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));

%Put a large weight on the pelvis CoordinateActuators, which act as the
%residual, or 'hand-of-god', forces which we would like to keep as small
%as possible.
for i = 0:inverseModel.updForceSet().getSize()-1
   forcePath = inverseModel.updForceSet().get(i).getAbsolutePathString();
   if contains(string(forcePath),'pelvis')
       effort.setWeightForControl(forcePath,10);
   else
       effort.setWeightForControl(forcePath,1);
   end
   %Cleanup
   clear forcePath
end

%Set other parameters
effort.setWeight(controlEffortWeight);
effort.setExponent(2);
effort.setDivideByDisplacement(true);

%Access the CasADi solver to configure
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(50);
solver.set_optim_max_iterations(1000);
solver.set_optim_convergence_tolerance(1e-3); %slightly reduced for hopefully some simulation brevity
solver.set_optim_constraint_tolerance(1e-3); %slightly reduced for hopefully some simulation brevity

%Save problem to text to review if necessary
study.print('gaitTracking_fromIK.omoco');
    
%Solve problem
clc
trackingSolution = study.solve();

%%%%% Reducing the convergence and constraint tolerance from the default (I
%%%%% think) 1e-4 to 1e-3 alowed the probem to converge a little quicker in
%%%%% about an hour.

%Compare tracking solution to IK

%Load in tracked data
trackingData_fromIK = importdata('gaitTracking_fromIK_solution.sto');

%Extract time
trackingResults_fromIK.time = trackingData_fromIK.data(:,1);

%Extract kinematic data
for cc = 1:length(trackingData_fromIK.colheaders)
    currState = trackingData_fromIK.colheaders{cc};
    %Check for kinematic data
    if contains(currState,'/jointset') && contains(currState,'/value')
        %Identify joint coordinate name
        splitStr = strsplit(currState,'/');
        currCoord = splitStr{4};
        %Extract data
        trackingResults_fromIK.kinematics.(char(currCoord)) = trackingData_fromIK.data(:,cc);
        %Cleanup
        clear currCoord splitStr
    end
end
clear cc

%Plot
figure; hold on
for pp = 1:length(coordPlot)
    subplot(4,3,pp); hold on
    %IK result
    plot(ikResults.time,ikResults.kinematics.(coordPlot{pp}),'k','LineWidth',1.5);
    %Inverse result
    plot(inverseResults_fromIK.time,rad2deg(inverseResults_fromIK.kinematics.(coordPlot{pp})),'r:','LineWidth',1.5);
    %Tracking result
    plot(trackingResults_fromIK.time,rad2deg(trackingResults_fromIK.kinematics.(coordPlot{pp})),'b--','LineWidth',1.5);
    %Set title
    title(coordPlotName{pp});
    %Set x-limits
    ax = gca; ax.XLim = [ikResults.time(1) ikResults.time(end)]; clear ax
end
clear pp

%%%%% With just state tracking and control effort (as a default parameter)
%%%%% the kinematics remain quite consistent from IK - inverse tracking -
%%%%% state tracking. Adding the additional cost functions may be something
%%%%% to build from this as a guess once it's converged.

%We can also compare the torque controls from the inverse simulation to the
%tracking simulation to see if these changed. In particular the pelvis
%actuators given a varied cost function was applied to these.

%Extract controls data. 
%Note that this assumes that the controls column headers are ordered the
%same across the inverse and tracking data
for cc = 1:length(inverseData_fromIK.colheaders)
    currState = inverseData_fromIK.colheaders{cc};
    %Check for forceset torque control data
    if contains(currState,'/forceset')
        %Identify joint coordinate name
        splitStr = strsplit(currState,'/');
        currCon = splitStr{3};
        %Extract data
        inverseResults_fromIK.controls.(char(currCon)) = inverseData_fromIK.data(:,cc);
        trackingResults_fromIK.controls.(char(currCon)) = trackingData_fromIK.data(:,cc);
        %Cleanup
        clear currCon splitStr
    end
end
clear cc

%Create plots to compare
conPlot = [{'tau_lumbar_extension'}; {'tau_lumbar_bending'}; {'tau_lumbar_rotation'};
    {'tau_pelvis_tilt'}; {'tau_pelvis_rotation'}; {'tau_pelvis_list'};
    {'tau_pelvis_tx'}; {'tau_pelvis_ty'}; {'tau_pelvis_tz'};
    {'tau_hip_flexion_r'}; {'tau_hip_adduction_r'}; {'tau_hip_rotation_r'};
    {'tau_knee_angle_r'}; {'tau_ankle_angle_r'}];
conPlotName = [{'Lumbar Extension'}; {'Lumbar Bending'}; {'Lumbar Rotation'};
    {'Pelvis Tilt'}; {'Pelvis Rotation'}; {'Pelvis List'};
    {'Pelvis Tx'}; {'Pelvis Ty'}; {'Pelvis Tz'};
    {'R. Hip Flexion'}; {'R. Hip Adduction'}; {'R. Hip Rotation'};
    {'R. Knee Angle'}; {'R. Ankle Angle'}];

%Plot
figure; hold on
for pp = 1:length(conPlot)
    subplot(5,3,pp); hold on
    %Inverse result
% % %     plot(inverseResults_fromIK.time,inverseResults_fromIK.controls.(conPlot{pp}),'r:','LineWidth',1.5);
    %Tracking result
    plot(trackingResults_fromIK.time,trackingResults_fromIK.controls.(conPlot{pp}),'b--','LineWidth',1.5);
    %Set title
    title(conPlotName{pp});
    %Set x-limits
    ax = gca; ax.XLim = [ikResults.time(1) ikResults.time(end)]; clear ax
end
clear pp

%%%%% The controls follow a similar pattern, but the states tracking
%%%%% options are clearly a lot smoother - this is probably to do with the
%%%%% approach in a forward vs. inverse simulation as the main factor.
%%%%% Nonetheless, we would need to examine the pelvis torques and forces
%%%%% to actually see the magnitude of these residuals, though the controls
%%%%% are quite low. Multiplying these by the optimal force of 1000 equates
%%%%% to a maximum of 100N/Nm seemingly for the residuals. Still high, but
%%%%% perhaps could be reduced even further if the optimal force of these
%%%%% actuators was substantially dropped penalising their use even more.

%% Re-run the same states tracking problem from before, but include the contact
%  spheres and don't attach the GRF data. See how this changes the solution
%  and what the predicted GRFs look like

%Set the global tracking weights for each aspect of the problem
controlEffortWeight = 0.001; %default
stateTrackingWeight = 5;
% % % GRFTrackingWeight   = 5;

%Define the motion tracking problem
grfTrack = MocoTrack();
grfTrack.setName('gaitTracking_withoutGRF_fromIK');

%Set the kinematics
grfTrack.setStatesReference(TableProcessor([pwd,'\ikResults_states.sto']));
grfTrack.set_states_global_tracking_weight(1.0);
grfTrack.set_allow_unused_references(true); 

%Set the model

%Need to add the contact spheres and geometry back into the inverse model
%to allow for GRF tracking
%The contact spheres and contact geometry need to be copied back to the
%tracking model
trackGRFmodel = ModelProcessor(inverseModel).process();

%Copy the contact geometry set from the scaled model to the new model
trackGRFmodel.set_ContactGeometrySet(scaledModelMuscle.getContactGeometrySet());

%Copy the spheres across
for ii = 0:scaledModelMuscle.updForceSet().getSize()-1
    %take the force object if it is a contact object
    if contains(char(scaledModelMuscle.updForceSet().get(ii).getName()),'contact')
        trackGRFmodel.updForceSet().cloneAndAppend(scaledModelMuscle.updForceSet().get(ii));        
    end
end

%Finalise model connections
trackGRFmodel.finalizeConnections();

%Set model in problem
grfTrack.setModel(ModelProcessor(trackGRFmodel));

%Initialise the tracking object to recieve a pre-configured MocoStudy
%object based on the current track settings above. Use this to customise
%the problem beyond the track interface.
grfStudy = grfTrack.initialize();

% Get a reference to the study problem
grfProblem = grfStudy.updProblem();

%Set state weights within problem
%Generic weights for everything right now
statesWeights = MocoWeightSet();
%Get state variable names
trackGRFmodel_state = trackGRFmodel.initSystem();
stateVars = trackGRFmodel.getStateVariableNames();
%Convert to string array while only taking jointset states
r = 1;
for ss = 0:stateVars.getSize()-1
    currState = char(stateVars.get(ss));
    if contains(currState,'/jointset')
        jointStates{r,1} = currState;
        r = r + 1;
    end
end
clear ss
%Append each state weight to goal
%This sets all non pelvis weights to a value of 5 and the pelvis values to
%1, as a basic means to allow these to be altered to minimise residual
%pelvis use. This is a simplistic approach but a start to see how it works.
for ss = 1:length(jointStates)
    if contains(jointStates{ss},'pelvis')
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},1.0));
    else
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},5.0));
    end
end
clear ss
%Set states goal in study
statesGoal = MocoStateTrackingGoal.safeDownCast(grfProblem.updGoal('state_tracking'));
statesGoal.setWeightSet(statesWeights);
statesGoal.setWeight(stateTrackingWeight);

%Get a reference to the MocoControlGoal that is added to every MocoTrack
%problem by default and change the weight
effort = MocoControlGoal.safeDownCast(grfProblem.updGoal('control_effort'));

%Put a large weight on the pelvis CoordinateActuators, which act as the
%residual, or 'hand-of-god', forces which we would like to keep as small
%as possible.
%Slight edits to this iteration due to contact spheres now being included
%in the forceset
for i = 0:trackGRFmodel.updForceSet().getSize()-1
   forcePath = trackGRFmodel.updForceSet().get(i).getAbsolutePathString();
   if contains(string(forcePath),'tau')
       if contains(string(forcePath),'pelvis')
           effort.setWeightForControl(forcePath,10);
       else
           effort.setWeightForControl(forcePath,1);
       end
   end
   %Cleanup
   clear forcePath
end

%Set other parameters
effort.setWeight(controlEffortWeight);
effort.setExponent(2);
effort.setDivideByDisplacement(true);

%Access the CasADi solver to configure
grfSolver = grfStudy.initCasADiSolver();
grfSolver.set_num_mesh_intervals(50);
grfSolver.set_optim_max_iterations(1000);
grfSolver.set_optim_convergence_tolerance(1e-3); %slightly reduced for hopefully some simulation brevity
grfSolver.set_optim_constraint_tolerance(1e-3); %slightly reduced for hopefully some simulation brevity

%Set the guess as the previous tracking solution
grfSolver.setGuess(trackingSolution);

%Save problem to text to review if necessary
grfStudy.print('gaitTracking_withoutGRF_fromIK.omoco');
    
%Solve problem
clc
trackingSolution_withoutGRF = grfStudy.solve();

%Compare states tracking solution original and then without the GRF

%Load in tracked data
trackingData_withoutGRF_fromIK = importdata('gaitTracking_withoutGRF_fromIK_solution.sto');

%Extract time
trackingResults_withoutGRF_fromIK.time = trackingData_withoutGRF_fromIK.data(:,1);

%Extract kinematic data
for cc = 1:length(trackingData_withoutGRF_fromIK.colheaders)
    currState = trackingData_withoutGRF_fromIK.colheaders{cc};
    %Check for kinematic data
    if contains(currState,'/jointset') && contains(currState,'/value')
        %Identify joint coordinate name
        splitStr = strsplit(currState,'/');
        currCoord = splitStr{4};
        %Extract data
        trackingResults_withoutGRF_fromIK.kinematics.(char(currCoord)) = trackingData_withoutGRF_fromIK.data(:,cc);
        %Cleanup
        clear currCoord splitStr
    end
end
clear cc

%Add pelvis translations to plot as these are important
coordPlot = [{'lumbar_extension'}; {'lumbar_bending'}; {'lumbar_rotation'};
    {'pelvis_tilt'}; {'pelvis_rotation'}; {'pelvis_list'};
    {'pelvis_tx'}; {'pelvis_ty'}; {'pelvis_tz'};
    {'hip_flexion_r'}; {'hip_adduction_r'}; {'hip_rotation_r'};
    {'knee_angle_r'}; {'ankle_angle_r'}];
coordPlotName = [{'Lumar Extension'}; {'Lumbar Bending'}; {'Lumbar Rotation'};
    {'Pelvis Tilt'}; {'Pelvis Rotation'}; {'Pelvis List'};
    {'Pelvis Translation X'}; {'Pelvis Translation Y'}; {'Pelvis Translation Z'};
    {'R. Hip Flexion'}; {'R. Hip Adduction'}; {'R. Hip Rotation'};
    {'R. Knee Angle'}; {'R. Ankle Angle'}];

%Plot
figure; hold on
for pp = 1:length(coordPlot)
    subplot(5,3,pp); hold on
    %IK result
    plot(ikResults.time,ikResults.kinematics.(coordPlot{pp}),'k','LineWidth',1.5);
    if contains(coordPlotName{pp},'Pelvis Translation') %don't convert from radians
        %Original tracking result
        plot(trackingResults_fromIK.time,trackingResults_fromIK.kinematics.(coordPlot{pp}),'r:','LineWidth',1.5);
        %Tracking result without GRF
        plot(trackingResults_withoutGRF_fromIK.time,trackingResults_withoutGRF_fromIK.kinematics.(coordPlot{pp}),'b--','LineWidth',1.5);
    else
        %Original tracking result
        plot(trackingResults_fromIK.time,rad2deg(trackingResults_fromIK.kinematics.(coordPlot{pp})),'r:','LineWidth',1.5);
        %Tracking result without GRF
        plot(trackingResults_withoutGRF_fromIK.time,rad2deg(trackingResults_withoutGRF_fromIK.kinematics.(coordPlot{pp})),'b--','LineWidth',1.5);
    end
    %Set title
    title(coordPlotName{pp});
    %Set x-limits
    ax = gca; ax.XLim = [ikResults.time(1) ikResults.time(end)]; clear ax
end
clear pp

%%%%% From a kinematics perspective the two tracking solutions look fairly
%%%%% similar. There are some small differences in pelvis rotations, but
%%%%% the biggest difference is in the pelvis Y translation, where without
%%%%% the GRF applied the pelvis is lower. It starts lower and then
%%%%% translates up but still remains lower. Checking the controls, but
%%%%% also the predicted GRFs will help figure out more about the
%%%%% solutions.

%Extract controls data. 
%Note that this assumes that the controls column headers are ordered the
%same across the inverse and tracking data
for cc = 1:length(trackingData_withoutGRF_fromIK.colheaders)
    currState = trackingData_withoutGRF_fromIK.colheaders{cc};
    %Check for forceset torque control data
    if contains(currState,'/forceset')
        %Identify joint coordinate name
        splitStr = strsplit(currState,'/');
        currCon = splitStr{3};
        %Extract data
        trackingResults_withoutGRF_fromIK.controls.(char(currCon)) = trackingData_withoutGRF_fromIK.data(:,cc);
        %Cleanup
        clear currCon splitStr
    end
end
clear cc

%Plot
figure; hold on
for pp = 1:length(conPlot)
    subplot(5,3,pp); hold on
    %Original Tracking result
    plot(trackingResults_fromIK.time,trackingResults_fromIK.controls.(conPlot{pp}),'r:','LineWidth',1.5);
    %Tracking result without GRF
    plot(trackingResults_withoutGRF_fromIK.time,trackingResults_withoutGRF_fromIK.controls.(conPlot{pp}),'b--','LineWidth',1.5);
    %Set title
    title(conPlotName{pp});
    %Set x-limits
    ax = gca; ax.XLim = [ikResults.time(1) ikResults.time(end)]; clear ax
end
clear pp

%%%%% Controls are definitely noisier and larger (for the most part) in the
%%%%% simulation without GRF to maintain the kinematic states. This is
%%%%% potentially due to the mismatch between the GRF data applied vs. that
%%%%% being produced by the contact spheres (i.e. the actuators are more so
%%%%% needed to maintain the original kinematics in the face of altered
%%%%% external forces).

%Compare the experimental GRF data to that predicted by the contact spheres
%with the tracked kinematic states (albeit slightly altered kinematics)

% % % %Create structures for force
% % % contact_r = StdVectorString();
% % % contact_l = StdVectorString();
% % % contact_r.add('/forceset/contactHeel_r');
% % % contact_r.add('/forceset/contactMH1_r');
% % % contact_r.add('/forceset/contactMH3_r');
% % % contact_r.add('/forceset/contactMH5_r');
% % % contact_r.add('/forceset/contactOtherToes_r');
% % % contact_r.add('/forceset/contactHallux_r');
% % % contact_l.add('/forceset/contactHeel_l');
% % % contact_l.add('/forceset/contactMH1_l');
% % % contact_l.add('/forceset/contactMH3_l');
% % % contact_l.add('/forceset/contactMH5_l');
% % % contact_l.add('/forceset/contactOtherToes_l');
% % % contact_l.add('/forceset/contactHallux_l');
% % % 
% % % %Get forces and write to file
% % % externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(trackGRFmodel,trackingSolution_withoutGRF,contact_r,contact_l);
% % % opensimMoco.writeTableToFile(externalForcesTableFlat,'gaitTracking_withoutGRF_fromIK_solution_GRF.sto');

%%%%% This approach doesn't seem to work for some reason...
%%%%% -nan(ind) outputs coming from the process

% % % outputPaths = StdVectorString();
% % % outputPaths.add('/forceset/contactMH1_r.*');
% % % 
% % % %outputTable is TimeSeriesTable with time vector equal to the MocoTrajectory
% % % clc
% % % outputTable = grfStudy.analyze(trackingSolution_withoutGRF, outputPaths)

%% Build on the existing states tracking simulation by removing the external
%  loads and including a GRF prediction goal. This builds on the last
%  tracking simulation with only one extra goal, that being the GRF contact
%  tracking

%Set the global tracking weights for each aspect of the problem
controlEffortWeight = 0.001; %default
stateTrackingWeight = 5;
GRFTrackingWeight   = 1;

%Define the motion tracking problem
grfTrack = MocoTrack();
grfTrack.setName('gaitTracking_withGRF_fromIK');

%Set the kinematics
grfTrack.setStatesReference(TableProcessor([pwd,'\ikResults_states.sto']));
grfTrack.set_states_global_tracking_weight(1.0);
grfTrack.set_allow_unused_references(true); 

%Set the model

%Need to add the contact spheres and geometry back into the inverse model
%to allow for GRF tracking
%The contact spheres and contact geometry need to be copied back to the
%tracking model
trackGRFmodel = ModelProcessor(inverseModel).process();

%Copy the contact geometry set from the scaled model to the new model
trackGRFmodel.set_ContactGeometrySet(scaledModelMuscle.getContactGeometrySet());

%Copy the spheres across
for ii = 0:scaledModelMuscle.updForceSet().getSize()-1
    %take the force object if it is a contact object
    if contains(char(scaledModelMuscle.updForceSet().get(ii).getName()),'contact')
        trackGRFmodel.updForceSet().cloneAndAppend(scaledModelMuscle.updForceSet().get(ii));        
    end
end

%Finalise model connections
trackGRFmodel.finalizeConnections();

%Set model in problem
grfTrack.setModel(ModelProcessor(trackGRFmodel));

%Initialise the tracking object to recieve a pre-configured MocoStudy
%object based on the current track settings above. Use this to customise
%the problem beyond the track interface.
grfStudy = grfTrack.initialize();

% Get a reference to the study problem
grfProblem = grfStudy.updProblem();

%Set state weights within problem
%Generic weights for everything right now
statesWeights = MocoWeightSet();
%Get state variable names
trackGRFmodel_state = trackGRFmodel.initSystem();
stateVars = trackGRFmodel.getStateVariableNames();
%Convert to string array while only taking jointset states
r = 1;
for ss = 0:stateVars.getSize()-1
    currState = char(stateVars.get(ss));
    if contains(currState,'/jointset')
        jointStates{r,1} = currState;
        r = r + 1;
    end
end
clear ss
%Append each state weight to goal
%This sets all non pelvis weights to a value of 5 and the pelvis values to
%1, as a basic means to allow these to be altered to minimise residual
%pelvis use. This is a simplistic approach but a start to see how it works.
for ss = 1:length(jointStates)
    if contains(jointStates{ss},'pelvis')
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},1.0));
    else
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},5.0));
    end
end
clear ss
%Set states goal in study
statesGoal = MocoStateTrackingGoal.safeDownCast(grfProblem.updGoal('state_tracking'));
statesGoal.setWeightSet(statesWeights);
statesGoal.setWeight(stateTrackingWeight);

%Get a reference to the MocoControlGoal that is added to every MocoTrack
%problem by default and change the weight
effort = MocoControlGoal.safeDownCast(grfProblem.updGoal('control_effort'));

%Put a large weight on the pelvis CoordinateActuators, which act as the
%residual, or 'hand-of-god', forces which we would like to keep as small
%as possible.
%Slight edits to this iteration due to contact spheres now being included
%in the forceset
for i = 0:trackGRFmodel.updForceSet().getSize()-1
   forcePath = trackGRFmodel.updForceSet().get(i).getAbsolutePathString();
   if contains(string(forcePath),'tau')
       if contains(string(forcePath),'pelvis')
           effort.setWeightForControl(forcePath,10);
       else
           effort.setWeightForControl(forcePath,1);
       end
   end
   %Cleanup
   clear forcePath
end

%Set other parameters
effort.setWeight(controlEffortWeight);
effort.setExponent(2);
effort.setDivideByDisplacement(true);

%Add a contact tracking goal

%Create the GRF tracking goal
contactTracking = MocoContactTrackingGoal('contact',GRFTrackingWeight);

%Set the external loads file
contactTracking.setExternalLoadsFile([pwd,'\Jog05_grf.xml']);

%Set the force names for the right foot
forceNamesRightFoot = StdVectorString();
forceNamesRightFoot.add('/forceset/contactHeel_r');
forceNamesRightFoot.add('/forceset/contactMH1_r');
forceNamesRightFoot.add('/forceset/contactMH3_r');
forceNamesRightFoot.add('/forceset/contactMH5_r');
forceNamesRightFoot.add('/forceset/contactOtherToes_r');
forceNamesRightFoot.add('/forceset/contactHallux_r');

%Set the force names for the left foot
forceNamesLeftFoot = StdVectorString();
forceNamesLeftFoot.add('/forceset/contactHeel_l');
forceNamesLeftFoot.add('/forceset/contactMH1_l');
forceNamesLeftFoot.add('/forceset/contactMH3_l');
forceNamesLeftFoot.add('/forceset/contactMH5_l');
forceNamesLeftFoot.add('/forceset/contactOtherToes_l');
forceNamesLeftFoot.add('/forceset/contactHallux_l');

%Add the tracking groups
%Right foot
trackRightGRF = MocoContactTrackingGoalGroup(forceNamesRightFoot,'calcn_r');
trackRightGRF.append_alternative_frame_paths('/bodyset/toes_r');
contactTracking.addContactGroup(trackRightGRF);
%Left foot
trackLeftGRF = MocoContactTrackingGoalGroup(forceNamesLeftFoot,'calcn_l');
trackLeftGRF.append_alternative_frame_paths('/bodyset/toes_l');
contactTracking.addContactGroup(trackLeftGRF);

%Add to problem
grfProblem.addGoal(contactTracking);

%Access the CasADi solver to configure
grfSolver = grfStudy.initCasADiSolver();
grfSolver.set_num_mesh_intervals(50);
grfSolver.set_optim_max_iterations(1000);
grfSolver.set_optim_convergence_tolerance(1e-3); %slightly reduced for hopefully some simulation brevity
grfSolver.set_optim_constraint_tolerance(1e-3); %slightly reduced for hopefully some simulation brevity

%Set the guess as the previous tracking solution
grfSolver.setGuess(trackingSolution);

%Save problem to text to review if necessary
grfStudy.print('gaitTracking_withGRF_fromIK.omoco');
    
%Solve problem
clc
trackingSolution_withGRF = grfStudy.solve();
% % % trackingSolution_withGRF.unseal(); 

%%%%% With altered scaling parameters ensuring the spheres contact the
%%%%% ground during IK, the GRF tracking optimisation works better but
%%%%% still reached 1000 iterations before converging. The unscaled
%%%%% objective function is sitting around 2.2 with the majority of this
%%%%% coming from the contact tracking. The motion looks correct which is
%%%%% reflected by the low states tracking cost. The problem looked liked
%%%%% it was getting close to converging with low values for other
%%%%% parameters too...

%Load in and compare kinematics between the states tracking and GRF
%tracking simulations

%Load data
trackingData_withGRF_fromIK = importdata('gaitTracking_withGRF_fromIK_solution.sto');

%Extract time
trackingResults_withGRF_fromIK.time = trackingData_withGRF_fromIK.data(:,1);

%Extract kinematic data
for cc = 1:length(trackingData_withGRF_fromIK.colheaders)
    currState = trackingData_withGRF_fromIK.colheaders{cc};
    %Check for kinematic data
    if contains(currState,'/jointset') && contains(currState,'/value')
        %Identify joint coordinate name
        splitStr = strsplit(currState,'/');
        currCoord = splitStr{4};
        %Extract data
        trackingResults_withGRF_fromIK.kinematics.(char(currCoord)) = trackingData_withGRF_fromIK.data(:,cc);
        %Cleanup
        clear currCoord splitStr
    end
end
clear cc

%Plot
figure; hold on
for pp = 1:length(coordPlot)
    subplot(5,3,pp); hold on
    %IK result
    plot(ikResults.time,ikResults.kinematics.(coordPlot{pp}),'k','LineWidth',1.5);
    if contains(coordPlotName{pp},'Pelvis Translation') %don't convert from radians
% % %         %Original tracking result
% % %         plot(trackingResults_fromIK.time,trackingResults_fromIK.kinematics.(coordPlot{pp}),'r:','LineWidth',1.5);
        %Tracking result with GRF
        plot(trackingResults_withGRF_fromIK.time,trackingResults_withGRF_fromIK.kinematics.(coordPlot{pp}),'b--','LineWidth',1.5);
    else
% % %         %Original tracking result
% % %         plot(trackingResults_fromIK.time,rad2deg(trackingResults_fromIK.kinematics.(coordPlot{pp})),'r:','LineWidth',1.5);
        %Tracking result with GRF
        plot(trackingResults_withGRF_fromIK.time,rad2deg(trackingResults_withGRF_fromIK.kinematics.(coordPlot{pp})),'b--','LineWidth',1.5);
    end
    %Set title
    title(coordPlotName{pp});
    %Set x-limits
    ax = gca; ax.XLim = [ikResults.time(1) ikResults.time(end)]; clear ax
end
clear pp

%%%%% From a kinematics perspective the GRF tracking simulation is pretty
%%%%% similar to the IK results - lumbar, hip, knee and ankle motion are
%%%%% almost identical. Pelvis X and Z translations are pretty identical -
%%%%% the main, albeit small-ish differences are in pelvis tilt and pelvis
%%%%% Y translation.

%Extract controls data. 
%Note that this assumes that the controls column headers are ordered the
%same across the inverse and tracking data
for cc = 1:length(trackingData_withGRF_fromIK.colheaders)
    currState = trackingData_withGRF_fromIK.colheaders{cc};
    %Check for forceset torque control data
    if contains(currState,'/forceset')
        %Identify joint coordinate name
        splitStr = strsplit(currState,'/');
        currCon = splitStr{3};
        %Extract data
        trackingResults_withGRF_fromIK.controls.(char(currCon)) = trackingData_withGRF_fromIK.data(:,cc);
        %Cleanup
        clear currCon splitStr
    end
end
clear cc

%Plot
figure; hold on
for pp = 1:length(conPlot)
    subplot(5,3,pp); hold on
    %Original Tracking result
    plot(trackingResults_fromIK.time,trackingResults_fromIK.controls.(conPlot{pp}),'r:','LineWidth',1.5);
    %Tracking result without GRF
    plot(trackingResults_withGRF_fromIK.time,trackingResults_withGRF_fromIK.controls.(conPlot{pp}),'b--','LineWidth',1.5);
    %Set title
    title(conPlotName{pp});
    %Set x-limits
    ax = gca; ax.XLim = [ikResults.time(1) ikResults.time(end)]; clear ax
end
clear pp

%%%%% Controls data from the GRF tracking simulation are a bit noisier,
%%%%% particularly around the middle where GRFs are highest. The residuals
%%%%% of the pelvis are quite high, but this may not be problematic given
%%%%% that these are changing the pelvis kinematics?

%%%%% ISSUE WITH CALCULATING GRF'S STILL PERSISTS, SO IT'S DIFFICULT TO
%%%%% COMPARE WITH THE EXPERIMENTAL DATA!

%Extract predicted GRFs and compare to experimental

%Create structures for force
contact_r = StdVectorString();
contact_l = StdVectorString();
contact_r.add('/forceset/contactHeel_r');
contact_r.add('/forceset/contactMH1_r');
contact_r.add('/forceset/contactMH3_r');
contact_r.add('/forceset/contactMH5_r');
contact_r.add('/forceset/contactOtherToes_r');
contact_r.add('/forceset/contactHallux_r');
contact_l.add('/forceset/contactHeel_l');
contact_l.add('/forceset/contactMH1_l');
contact_l.add('/forceset/contactMH3_l');
contact_l.add('/forceset/contactMH5_l');
contact_l.add('/forceset/contactOtherToes_l');
contact_l.add('/forceset/contactHallux_l');

%Get forces and write to file
externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(trackGRFmodel,trackingSolution_withGRF,contact_r,contact_l);
opensimMoco.writeTableToFile(externalForcesTableFlat,'testGRF.sto');

%% Test a basic states tracking solution with experimental GRFs applied
%  but add in a frame distance constraint to keep the bodies of the foot
%  from penetrating the floor.

%Set the global tracking weights for each aspect of the problem
controlEffortWeight = 0.001; %default
stateTrackingWeight = 5;

%Define the motion tracking problem
trackFrame = MocoTrack();
trackFrame.setName('gaitTracking_withFrameDistance_fromIK');

%Set the kinematics
trackFrame.setStatesReference(TableProcessor([pwd,'\ikResults_states.sto']));
trackFrame.set_states_global_tracking_weight(1.0);
trackFrame.set_allow_unused_references(true); 

%Get the scaled muscle model to use in the simulation
trackFrameModel = Model('scaledModelMuscle.osim');

%Clear this models forceset
trackFrameModel.updForceSet().clearAndDestroy();

%Appropriate the contact sphere forces back into the model from an existing
%model iteration
%Copy the spheres across
for ii = 0:scaledModelMuscle.updForceSet().getSize()-1
    %take the force object if it is a contact object
    if contains(char(scaledModelMuscle.updForceSet().get(ii).getName()),'contact')
        trackFrameModel.updForceSet().cloneAndAppend(scaledModelMuscle.updForceSet().get(ii));        
    end
end

%Set these contact spheres to not apply force in the current iteration
for ii = 0:trackFrameModel.updForceSet().getSize()-1
    trackFrameModel.updForceSet().get(ii).set_appliesForce(false);    
end
clear ii

%Loop through and add coordinate actuators back into the models forceset to
%drive the motion.
for cc = 0:trackFrameModel.getCoordinateSet().getSize()-1
    
    %Create the coordinate actuator
    actu = CoordinateActuator();
    
    %Get name of the current coordinate
    currCoord = trackFrameModel.getCoordinateSet().get(cc).getName();
    
    %Set actuator coordinate
    actu.setCoordinate(trackFrameModel.getCoordinateSet().get(cc));
    
    %Set actuator name
    actu.setName(['tau_',char(currCoord)]);
    
    %Set optimal force
    actu.setOptimalForce(1000);
    
    %Set control levels    
    actu.setMaxControl(inf);
    actu.setMinControl(-inf);
    
    %Add actuator to model
    trackFrameModel.updForceSet().cloneAndAppend(actu);
    
end
clear cc

%Finalise model connections
trackFrameModel.finalizeConnections();

%Set model in inverse problem
trackFrameModel_modelProcessor = ModelProcessor(trackFrameModel);
trackFrameModel_modelProcessor.append(ModOpAddExternalLoads([pwd,'\Jog05_grf.xml']));
trackFrame.set_model(trackFrameModel_modelProcessor);

%Initialise the tracking object to recieve a pre-configured MocoStudy
%object based on the current track settings above. Use this to customise
%the problem beyond the track interface.
trackStudy = trackFrame.initialize();

% Get a reference to the study problem
trackProblem = trackStudy.updProblem();

%Set state weights within problem
%Generic weights for everything right now
statesWeights = MocoWeightSet();
%Get state variable names
trackFrameModel_state = trackFrameModel.initSystem();
stateVars = trackFrameModel.getStateVariableNames();
%Convert to string array while only taking jointset states
r = 1;
for ss = 0:stateVars.getSize()-1
    currState = char(stateVars.get(ss));
    if contains(currState,'/jointset')
        jointStates{r,1} = currState;
        r = r + 1;
    end
end
clear ss
%Append each state weight to goal
%This sets all non pelvis weights to a value of 5 and the pelvis values to
%1, as a basic means to allow these to be altered to minimise residual
%pelvis use. This is a simplistic approach but a start to see how it works.
for ss = 1:length(jointStates)
    if contains(jointStates{ss},'pelvis')
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},1.0));
    else
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},5.0));
    end
end
clear ss
%Set states goal in study
statesGoal = MocoStateTrackingGoal.safeDownCast(trackProblem.updGoal('state_tracking'));
statesGoal.setWeightSet(statesWeights);
statesGoal.setWeight(stateTrackingWeight);

%Get a reference to the MocoControlGoal that is added to every MocoTrack
%problem by default and change the weight
effort = MocoControlGoal.safeDownCast(trackProblem.updGoal('control_effort'));

%Put a large weight on the pelvis CoordinateActuators, which act as the
%residual, or 'hand-of-god', forces which we would like to keep as small
%as possible.
for i = 0:trackFrameModel.updForceSet().getSize()-1
   forcePath = trackFrameModel.updForceSet().get(i).getAbsolutePathString();
   if contains(string(forcePath),'tau')
       if contains(string(forcePath),'pelvis')
           effort.setWeightForControl(forcePath,10);
       else
           effort.setWeightForControl(forcePath,1);
       end
   end
   %Cleanup
   clear forcePath
end

%Set other parameters
effort.setWeight(controlEffortWeight);
effort.setExponent(2);
effort.setDivideByDisplacement(true);

%Set the frame distance constraints in the problem
distanceConstraint = MocoFrameDistanceConstraint();
distanceConstraint.setName('foot_ground_constraint');

%Add the frame pairs. The origins of the calcn and toes bodies lie pretty
%closely to the ground, so we just need to set how close to the ground we
%want these to be able to get. Currently set this to quite a low value to
%ensure they can get quite close.
distanceConstraint.addFramePair(MocoFrameDistanceConstraintPair('/bodyset/calcn_r','/ground',1e-10,Inf))
distanceConstraint.addFramePair(MocoFrameDistanceConstraintPair('/bodyset/calcn_l','/ground',1e-10,Inf))
distanceConstraint.addFramePair(MocoFrameDistanceConstraintPair('/bodyset/toes_r','/ground',1e-10,Inf))
distanceConstraint.addFramePair(MocoFrameDistanceConstraintPair('/bodyset/toes_l','/ground',1e-10,Inf))

%Set the distance constraint to be applied relative to the yaxis (i.e.
%vertical vector)
distanceConstraint.setProjection('vector');
distanceConstraint.setProjectionVector(Vec3(0,1,0));

%Add path constraint to problem
trackProblem.addPathConstraint(distanceConstraint);

%Access the CasADi solver to configure
trackSolver = trackStudy.initCasADiSolver();
trackSolver.set_num_mesh_intervals(50);
trackSolver.set_optim_max_iterations(1000);
trackSolver.set_optim_convergence_tolerance(1e-3); %slightly reduced for hopefully some simulation brevity
trackSolver.set_optim_constraint_tolerance(1e-3); %slightly reduced for hopefully some simulation brevity

%Set guess to previous state tracking solution
trackSolver.setGuess(trackingSolution);

%Save problem to text to review if necessary
trackStudy.print('gaitTracking_withFrameDistance_fromIK.omoco');
    
%Solve problem
clc
trackingSolution_withFrameDistance = trackStudy.solve();

%%%%% A couple of problems identified:
%%%%% (1) Non-fixed markers haven't been moved on the scaled model, which
%%%%% will impact IK results
%%%%% (2) In both the static and movement trials the positions and size of
%%%%% the contact spheres results in no penetration with the ground. This
%%%%% is particularly evident in the static trial given that the feet are
%%%%% off the ground when they are standing on it...
%%%%%
%%%%% The upper body markers aren't too far away from their experimentlal
%%%%% positions, so shifting the model down would increase the error on
%%%%% hese - potentially trying to fit the tracking markers to their wrong
%%%%% positions is causing issues, but I don't think that's the problem.
%%%%% The height scaling of the feet bodies could be an issue -- might need
%%%%% to test vertical scaling of bodies by adding virtual markers to the
%%%%% ground in the static trial...

%% Test what the state tracked kinematics look like from a GRF perspective

% % % %Create a force table from the tracking solution
% % % 
% % % %The contact spheres and contact geometry need to be copied back to the
% % % %tracking model
% % % trackGRFmodel = ModelProcessor(inverseModel).process();
% % % 
% % % %Copy the contact geometry set from the scaled model to the new model
% % % trackGRFmodel.set_ContactGeometrySet(scaledModelMuscle.getContactGeometrySet());
% % % 
% % % %Copy the spheres across
% % % for ii = 0:scaledModelMuscle.updForceSet().getSize()-1
% % %     %take the force object if it is a contact object
% % %     if contains(char(scaledModelMuscle.updForceSet().get(ii).getName()),'contact')
% % %         trackGRFmodel.updForceSet().cloneAndAppend(scaledModelMuscle.updForceSet().get(ii));        
% % %     end
% % % end
% % % 
% % % %Finalise model connections
% % % trackGRFmodel.finalizeConnections();
% % % 
% % % % % % %Create structures for force
% % % % % % contact_r = StdVectorString();
% % % % % % contact_l = StdVectorString();
% % % % % % contact_r.add('/forceset/contactHeel_r');
% % % % % % contact_r.add('/forceset/contactMH1_r');
% % % % % % contact_r.add('/forceset/contactMH3_r');
% % % % % % contact_r.add('/forceset/contactMH5_r');
% % % % % % contact_r.add('/forceset/contactOtherToes_r');
% % % % % % contact_r.add('/forceset/contactHallux_r');
% % % % % % contact_l.add('/forceset/contactHeel_l');
% % % % % % contact_l.add('/forceset/contactMH1_l');
% % % % % % contact_l.add('/forceset/contactMH3_l');
% % % % % % contact_l.add('/forceset/contactMH5_l');
% % % % % % contact_l.add('/forceset/contactOtherToes_l');
% % % % % % contact_l.add('/forceset/contactHallux_l');
% % % % % % 
% % % % % % %Get forces and write to file
% % % % % % externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(trackGRFmodel,trackingSolution,contact_r,contact_l);
% % % % % % opensimMoco.writeTableToFile(externalForcesTableFlat,'gaitTracking_fromIK_solution_GRF.sto');
% % % 
% % % %%%%% This above approach doesn't seem to work, could be the issue with
% % % %%%%% model not being consistent with the solution
% % % 
% % % %Attempt running a force reporter analysis with the tracked solution and
% % % %the updated model
% % % 
% % % %Create analysis tool
% % % grfAnalysis = AnalyzeTool();
% % % grfAnalysis.setName('forceAnalysis');
% % % 
% % % %Set times
% % % grfAnalysis.setInitialTime(startTime);
% % % grfAnalysis.setFinalTime(endTime);
% % % 
% % % %Set model
% % % trackGRFmodel.print('tempModel.osim');
% % % grfAnalysis.setModelFilename([pwd,'\tempModel.osim']);
% % % 
% % % %Initialise a force reporter analysis
% % % frAnalysis = ForceReporter();
% % % 
% % % %Add states data data to the analysis tool
% % % grfAnalysis.setStatesFileName('gaitTracking_fromIK_solution.sto');
% % % 
% % % %Add parameters to the body kinematics analysis
% % % frAnalysis.setStartTime(startTime);
% % % frAnalysis.setEndTime(endTime);
% % % frAnalysis.setInDegrees(false);
% % % 
% % % %Append analysis set
% % % grfAnalysis.getAnalysisSet().cloneAndAppend(frAnalysis);
% % % 
% % % %Print the analysis tool
% % % grfAnalysis.print('analysis.xml');
% % % 
% % % %Continually have trouble running analysis tools from within Matlab, so run
% % % %it using the command line triggers
% % % clc
% % % system('opensim-cmd run-tool analysis.xml');
% % % 
% % % %Clear temp setup files
% % % delete('analysis.xml'); delete('tempModel.osim');

%%%%% The forces here seem wrong too, so not sure if it's working
%%%%% correctly. May need to include within a tracking simulation to ensure
%%%%% that it is working properly.

%%

