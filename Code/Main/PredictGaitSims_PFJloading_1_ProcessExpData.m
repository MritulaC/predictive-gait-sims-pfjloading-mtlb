function PredictGaitSims_PFJloading_1_ProcessExpData

%%%%%%%%%% TO DO: function notes

% @author: Aaron Fox
% Centre for Sport Research, Deakin University
% aaron.f@deakin.edu.au
% 
%
% TO DO: add Fallise et al., Handsfield et al. & Rajagopal et al. references

    %% Set-up

    import org.opensim.modeling.*
    warning off

    %Set main directory
    mainDir = pwd;

    %Add supplementary code folder to path
    addpath(genpath('..\Supplementary'));
    
    %Add model geometry directory
    ModelVisualizer.addDirToGeometrySearchPaths('..\..\GenericModel\Geometry');
    
    %% Scale generic 3D walking model
    
    %Set up a scale tool
    scaleTool = ScaleTool();
    
    %Get mass from original RRA adjusted experimental model
    cd('..\..\ExpData\Static');    
    rraModel = Model('rraAdjustedOriginalModel.osim');
    modelMass = 0;
    for ii = 0:rraModel.getBodySet().getSize()-1
        modelMass = modelMass + rraModel.getBodySet().get(ii).getMass();        
    end
    clear ii
    
    %Set mass in scale tool
    scaleTool.setSubjectMass(modelMass);
    
    %Navigate to generic model directory
    cd('..\..\GenericModel'); genModelDir = [pwd,'\'];
    
    %Set generic model file
    scaleTool.getGenericModelMaker().setModelFileName([pwd,'\Moco_3Dwalking_TrunkMotion.osim']);
    
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
    cd('..\ExpData\Static');
    scaleTool.getMarkerPlacer().setMarkerFileName([pwd,'\static.trc']);
    scaleTool.getModelScaler().setMarkerFileName([pwd,'\static.trc']);
    
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
    scaleTool.getModelScaler().setOutputModelFileName([pwd,'\MocoScaledModel.osim']);
    scaleTool.getModelScaler().setOutputScaleFileName([pwd,'\ScaleSet.xml']);
    
    %Save and run scale tool
    scaleTool.print([pwd,'\scaleSetup.xml']);
    scaleTool.run();
    
    %Load in scaled model
    scaledModel = Model('MocoScaledModel.osim');
    
    %Scale muscle strength based on linear function presented in Handsfield
    %et al. (2014). This uses some convenience functions that are packaged
    %with the Rajagopal et al. (2015) gait model. Note that the height of
    %the generic model is 1.700 and the height of the experimental
    %participant is 1.759
    genModel = Model([genModelDir,'Moco_3Dwalking_TrunkMotion.osim']);
    scaledModelMuscle = scaleOptimalForceSubjectSpecific(genModel,scaledModel,1.700,1.759);
    scaledModelMuscle.print('MocoScaledModel_StrengthScaled.osim');
    
    %%
    
    %% Run a muscle drive state tracking simulation
    
    %Construct the MocoTrack tool.
    track = MocoTrack();
    track.setName("muscle_driven_state_tracking");

    %Construct a ModelProcessor and set it on the tool. The default
    %muscles in the model are replaced with optimization-friendly
    %DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    %parameters.
    modelProcessor = ModelProcessor([pwd,'\MocoScaledModel_StrengthScaled.osim']);
    cd('..\Gait');
    modelProcessor.append(ModOpAddExternalLoads([pwd,'\Jog05_grf.xml']));
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    %Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    %Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    %Set model processor
    track.setModel(modelProcessor);

    %Construct a TableProcessor of the coordinate data and pass it to the
    %track tool.
    
    %Before setting the table, the RRA kinematic file needs to be converted
    %to a states file that only has the joint kinematic values. This can be
    %done using a convenience function included with this repo.
    convertRRAtoStates('Jog05_rraKinematics.sto',rraModel,'coordinates.sto');    
   
    %Set the kinematics using the table processor
    track.setStatesReference(TableProcessor([pwd,'\coordinates.sto']));
    
    % This setting allows extra data columns contained in the states
    % reference that don't correspond to model coordinates.
    track.set_allow_unused_references(true);

    %Initial time, final time, and mesh interval.
    %Initial and final time set based on coordinates data file
    %Mesh inverval arbitralily set at the moment - 0.05 is the upper end
    %value recommended in Moco documentation for gait simulations (maybe
    %not running though?)
    %%%% TO DO: could set this with better process\
        %%%%% NOTE: The Falisse et al. study uses 50 mesh intervals for a
        %%%%% half gait cycle, so this would be 100 for the full cycle in
        %%%%% this instance...
    track.set_initial_time(Storage('coordinates.sto').getFirstTime());
    track.set_final_time(Storage('coordinates.sto').getLastTime());
    track.set_mesh_interval(0.05);
    
    % Instead of calling solve(), call initialize() to receive a pre-configured
    % MocoStudy object based on the settings above. Use this to customize the
    % problem beyond the MocoTrack interface.
    study = track.initialize();

    % Get a reference to the MocoControlGoal that is added to every MocoTrack
    % problem by default.
    problem = study.updProblem();
    effort = MocoControlGoal.safeDownCast(problem.updGoal('control_effort'));

    % Put a large weight on the pelvis CoordinateActuators, which act as the
    % residual, or 'hand-of-god', forces which we would like to keep as small
    % as possible.
    model = modelProcessor.process();
    model.initSystem();
    forceSet = model.getForceSet();
    for i = 0:forceSet.getSize()-1
       forcePath = forceSet.get(i).getAbsolutePathString();
       if contains(string(forcePath), 'pelvis')
           effort.setWeightForControl(forcePath, 10);
       end
    end
    
    %Set state weights within problem
    %Generic weights of 10 for everything right now
    statesWeights = MocoWeightSet();
    %Get state variable names
    stateVars = model.getStateVariableNames();
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
    for ss = 1:length(jointStates)
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},10));        
    end
    clear ss
    %Set states goal in study
    statesGoal = MocoStateTrackingGoal.safeDownCast(problem.updGoal('state_tracking'));
    statesGoal.setWeightSet(statesWeights);
    statesGoal.setWeight(10);

    %%%%%%% could adjust solution tolerance???

    %Solve the problem and write the solution to a Storage file.
    
    %Navigate to directory to store results
    cd('..\..'); mkdir('SimulationResults'); cd('SimulationResults');
    
    % Solve and visualize.
    muscleStateTrackingSolution = study.solve();
% % %     study.visualize(solution);
    
% % %     %Cleanup default solution file
% % %     delete MocoStudy_solution.sto
% % %     
    
    %% Re-run the above solver to get a 101 node solution
    
    solver = study.initCasADiSolver();
    
    %Update the mesh intervals
    solver.set_num_mesh_intervals(50);
    
    %Update the initial guess to use the original solution
    solver.setGuess(muscleStateTrackingSolution);
    
    %Re-run solver
    study.setName("muscle_driven_state_tracking_dense");
    muscleStateTrackingSolutionDense = study.solve();
    
    
    
    %%
    
    %% Optimise foot contact parameters using measured kinematics
    
    %Create new iteration of model
    contactModel = Model('MocoScaledModel_StrengthScaled.osim');
    
    %Add spheres using convenience function. This function also removes the
    %pelvis actuators from this model iteration.
    addSpheresToModel(contactModel,'MocoScaledModel_BaselineContact.osim','reduce');
    
    %%%% if the pelvis actuators get removed they conflict later when
    %%%% updating the problem (i.e. pelvis actuators are in the problem but
    %%%% not the trajectory - should try and fix this)
    
    %% Test a states and GRF tracking simulation (muscle driven)
    
    %Initialise tracking object
    track = MocoTrack();
    track.setName('statesWithGRF_tracking');
    
    %Setup a model to use in the tracking simulation
    trackModel = Model('MocoScaledModel_BaselineContact.osim');
% % %     %Remove the force set from this model to make it a torque problem
% % %     trackModel.updForceSet().clearAndDestroy();
% % %     %Add low level pelvis actuators to use in the problem
% % %     %Set general parameters
% % %     cAct = CoordinateActuator();
% % %     cAct.setMinControl(-1); cAct.setMaxControl(1); cAct.setOptimalForce(1);
% % %     %Add variants to model
% % %     cAct.setCoordinate(trackModel.getCoordinateSet().get('pelvis_tilt'));
% % %     cAct.setName('pelvis_tilt_reserve');
% % %     trackModel.updForceSet().cloneAndAppend(cAct);
% % %     cAct.setCoordinate(trackModel.getCoordinateSet().get('pelvis_list'));
% % %     cAct.setName('pelvis_list_reserve');
% % %     trackModel.updForceSet().cloneAndAppend(cAct);
% % %     cAct.setCoordinate(trackModel.getCoordinateSet().get('pelvis_rotation'));
% % %     cAct.setName('pelvis_rotation_reserve');
% % %     trackModel.updForceSet().cloneAndAppend(cAct);
% % %     cAct.setCoordinate(trackModel.getCoordinateSet().get('pelvis_tx'));
% % %     cAct.setName('pelvis_tx_reserve');
% % %     trackModel.updForceSet().cloneAndAppend(cAct);
% % %     cAct.setCoordinate(trackModel.getCoordinateSet().get('pelvis_ty'));
% % %     cAct.setName('pelvis_ty_reserve');
% % %     trackModel.updForceSet().cloneAndAppend(cAct);
% % %     cAct.setCoordinate(trackModel.getCoordinateSet().get('pelvis_tz'));
% % %     cAct.setName('pelvis_tz_reserve');
% % %     trackModel.updForceSet().cloneAndAppend(cAct);
% % %     %Finalise model connections
% % %     trackModel.finalizeConnections();
    
    % Construct a ModelProcessor and set it on the tool.
    modelProcessor = ModelProcessor(trackModel);
% % %     % Add CoordinateActuators to the model degrees-of-freedom. This
% % %     % ignores the pelvis coordinates which already have residual 
% % %     % CoordinateActuators.
% % %     modelProcessor.append(ModOpAddReserves(500));
    %Set tendon compliance to be ignored
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    %replace muscles in model with DegrooteFregly type
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    %ignore passive forces
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    %change active force width scale
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    %Set model processor
    track.setModel(modelProcessor);
    
    %Set tracking parameters    
    cd('..\Gait');
    track.setStatesReference(TableProcessor([pwd,'\coordinates.sto']));
    track.set_initial_time(Storage('coordinates.sto').getFirstTime());
    track.set_final_time(Storage('coordinates.sto').getLastTime());
    track.set_mesh_interval(0.02);
    track.set_allow_unused_references(true);
    track.set_track_reference_position_derivatives(true);
        
    %Initialise study object
    study = track.initialize();
    problem = study.updProblem();
    
    %Set state weights within problem
    statesWeights = MocoWeightSet();
    model = modelProcessor.process();
    model.initSystem();
    %Get state variable names
    stateVars = model.getStateVariableNames();
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
    for ss = 1:length(jointStates)
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},10));        
    end
    clear ss
    %Set states goal in study
    statesGoal = MocoStateTrackingGoal.safeDownCast(problem.updGoal('state_tracking'));
    statesGoal.setWeightSet(statesWeights);
    statesGoal.setWeight(10);
    
    %Put a large weight on the pelvis CoordinateActuators, which act as the
    %residual, or 'hand-of-god', forces which we would like to keep as small
    %as possible.
    effort = MocoControlGoal.safeDownCast(problem.updGoal("control_effort"));
% % %     effort.setWeight(10);
% % %     effort.setWeightForControl('/forceset/pelvis_tilt_reserve', 10);
% % %     effort.setWeightForControl('/forceset/pelvis_list_reserve', 10);
% % %     effort.setWeightForControl('/forceset/pelvis_rotation_reserve', 10);
% % %     effort.setWeightForControl('/forceset/pelvis_tx_reserve', 10);
% % %     effort.setWeightForControl('/forceset/pelvis_ty_reserve', 10);
% % %     effort.setWeightForControl('/forceset/pelvis_tz_reserve', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_tilt', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_list', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_rotation', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_tx', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_ty', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_tz', 10);
    
    %Add a contact tracking goal
    % Track the right and left vertical and fore-aft ground reaction forces.
    contactTracking = MocoContactTrackingGoal('contact', 10);
    %Set the external loads
    extLoads = ExternalLoads('Jog05_grf.xml',true);
% % %     extLoads.setDataFileName([pwd,'\Jog05_grf.mot']);    %update to include full path
    %Add external loads to goal
    contactTracking.setExternalLoads(extLoads);
    %Add the contact groups
    contactGroup_r = MocoContactTrackingGoalGroup();
    contactGroup_r.set_external_force_name('calcn_r');
    contactGroup_r.set_contact_force_paths(0,'/contactSphereHeel_r');
    contactGroup_r.set_contact_force_paths(1,'/contactSphereTarsals_r');
    contactGroup_r.set_contact_force_paths(2,'/contactSphereMetOneProx_r');
    contactGroup_r.set_contact_force_paths(3,'/contactSphereMetOneDist_r');
    contactGroup_r.set_contact_force_paths(4,'/contactSphereMetFiveProx_r');
    contactGroup_r.set_contact_force_paths(5,'/contactSphereMetFiveDist_r');
    contactTracking.addContactGroup(contactGroup_r);
    contactGroup_l = MocoContactTrackingGoalGroup();
    contactGroup_l.set_external_force_name('calcn_l');
    contactGroup_l.set_contact_force_paths(0,'/contactSphereHeel_l');
    contactGroup_l.set_contact_force_paths(1,'/contactSphereTarsals_l');
    contactGroup_l.set_contact_force_paths(2,'/contactSphereMetOneProx_l');
    contactGroup_l.set_contact_force_paths(3,'/contactSphereMetOneDist_l');
    contactGroup_l.set_contact_force_paths(4,'/contactSphereMetFiveProx_l');
    contactGroup_l.set_contact_force_paths(5,'/contactSphereMetFiveDist_l');
    contactTracking.addContactGroup(contactGroup_l);
    %Set parameters
    contactTracking.setProjection('plane');
    contactTracking.setProjectionVector(Vec3(0, 0, 1));
    problem.addGoal(contactTracking);
    
    %%%%% TO DO: navigate to appropriate location to store data - keep in
    %%%%% mind this could be impacted by the grf XML file not having a full
    %%%%% path to the .mot files where GRFs are stored...
    % % %     cd('..\Static');    %%%% fix this to a more appropriate location
    
    %Solve and visualize.
    trackSolution = study.solve();
% % %     study.visualize(trackSolution);
    
    %Create space to access contact sphere data
    contactSpheres_r = StdVectorString();
    contactSpheres_l = StdVectorString();
    contactSpheres_r.add('contactSphereHeel_r');
    contactSpheres_r.add('contactSphereTarsals_r');
    contactSpheres_r.add('contactSphereMetOneProx_r');
    contactSpheres_r.add('contactSphereMetOneDist_r');
    contactSpheres_r.add('contactSphereMetFiveProx_r');
    contactSpheres_r.add('contactSphereMetFiveDist_r');
    contactSpheres_l.add('contactSphereHeel_l');
    contactSpheres_l.add('contactSphereTarsals_l');
    contactSpheres_l.add('contactSphereMetOneProx_l');
    contactSpheres_l.add('contactSphereMetOneDist_l');
    contactSpheres_l.add('contactSphereMetFiveProx_l');
    contactSpheres_l.add('contactSphereMetFiveDist_l');

    %Convert and write forces data
    externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(trackModel, ...
                                 trackSolution, contactSpheres_r, contactSpheres_l);
    opensimMoco.writeTableToFile(externalForcesTableFlat, ...
                                 'statesWithGRF_tracking_solution_GRF.sto');
                             
	%Compare tracked to experimental GRFs
    
    %Use the external loads data to create a GRF force curve to compare the
    %optimisations to
    extGRF = importdata('Jog05_grf.mot');
    for ii = 1:length(extGRF.colheaders)
       GRFdata.(extGRF.colheaders{ii}) = extGRF.data(:,ii); 
    end
    clear extGRF
    %Trim and extract the force data to the kinematics times
    startTime = Storage('Jog05_rraKinematics.sto').getFirstTime();
    endTime = Storage('Jog05_rraKinematics.sto').getLastTime();
    startInd = find(GRFdata.time > startTime, 1) - 1;
    endInd = find(GRFdata.time > endTime, 1) - 1;
    %Set experimental GRF data variable
    %Structure is set as column 1 is the time
    %Structure is set as columns 2-4 are the right foot x,y,z values
    %Structure is set as columns 5-7 are the left foot x,y,z values
    expGRF(:,1) = GRFdata.time(startInd:endInd);
    expGRF(:,2) = GRFdata.calcn_r_ground_force_vx(startInd:endInd);
    expGRF(:,3) = GRFdata.calcn_r_ground_force_vy(startInd:endInd);
    expGRF(:,4) = GRFdata.calcn_r_ground_force_vz(startInd:endInd);
    expGRF(:,5) = GRFdata.calcn_l_ground_force_vx(startInd:endInd);
    expGRF(:,6) = GRFdata.calcn_l_ground_force_vy(startInd:endInd);
    expGRF(:,7) = GRFdata.calcn_l_ground_force_vz(startInd:endInd);
    
    %Get the tracked ground reaction force data in a useful format
    
    %Get column labels
    colLabels = externalForcesTableFlat.getColumnLabels();
    
    %Extract data
    for ii = 0:externalForcesTableFlat.getNumRows()-1
        %Get time data
        time(ii+1,1) = externalForcesTableFlat.getIndependentColumn().get(ii);
        %Get data
        for kk = 0:colLabels.size()-1
            grfData(ii+1,kk+1) = externalForcesTableFlat.getDependentColumn(colLabels.get(kk)).get(ii);
        end
        clear kk
    end
    clear ii
    
    %Plot and compare data
    figure; hold on
    
    %Right foot
    subplot(3,2,1); hold on
    plot(expGRF(:,1),expGRF(:,2),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,1),'r--','LineWidth',1);
    subplot(3,2,3); hold on
    plot(expGRF(:,1),expGRF(:,3),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,2),'r--','LineWidth',1);
    subplot(3,2,5); hold on
    plot(expGRF(:,1),expGRF(:,4),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,3),'r--','LineWidth',1);
    
    %Left foot
    subplot(3,2,2); hold on
    plot(expGRF(:,1),expGRF(:,5),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,7),'r--','LineWidth',1);
    subplot(3,2,4); hold on
    plot(expGRF(:,1),expGRF(:,6),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,8),'r--','LineWidth',1);
    subplot(3,2,6); hold on
    plot(expGRF(:,1),expGRF(:,7),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,9),'r--','LineWidth',1);
    
    
    %% Original test optimisation code...
    
    %Set starting points for sphere optimisation
    x0 = [0.0157484;-0.0109827; %heelSphere_r x and y pos
        0.0753592;0.0226294; %tarsalsSphere_r x and y pos
        0.184621;-0.0278901; %metOneProxSphere_r x and y pos
        0.254175;-0.0166661; %metOneDistSphere_r x and y pos
        0.140891;0.0463663; %metFiveProxSphere_r x and y pos
        0.198869;0.0629414; %metFiveDistSphere_r x and y pos
        0.0157484;0.0109827; %heelSphere_l x and y pos
        0.0753592;-0.0226294; %tarsalsSphere_l x and y pos
        0.184621;0.0278901; %metOneProxSphere_l x and y pos
        0.254175;0.0166661; %metOneDistSphere_l x and y pos
        0.140891;-0.0463663; %metFiveProxSphere_l x and y pos
        0.198869;-0.0629414; %metFiveDistSphere_l x and y pos
        ones(12,1)*0.032]; %all sphere radii
    
    %Set bounds for parameters. Location points are allowed to move 0.1 in
    %positive or negative direction, while radii are limited to 0.001 to
    %0.064.
    xUB = [x0(1:24)+0.1;
        ones(12,1)*0.001];
    xLB = [x0(1:24)-0.1;
        ones(12,1)*0.064];
    
    %Set options for optimisation (deafult, but can be changed with access
    %to the structure easily)
    options = optimset('fminsearch');
    
    %Setup a model as a global variable to pass between functions
    optimModel = Model('MocoScaledModel_BaselineContact.osim');
    global optimModel
    %Clear the force set from the model as we're not interested in muscles
    %and that will speed/clean up the process
    optimModel.updForceSet().clearAndDestroy();
    optimModel.finalizeConnections();
    
    %Set-up inputs for use within function. Set these as global as well to
    %pass between functions
    global Input
    
    %Use the external loads data to create a GRF force curve to compare the
    %optimisations to
    cd('..\Gait');
    extGRF = importdata('Jog05_grf.mot');
    for ii = 1:length(extGRF.colheaders)
       GRFdata.(extGRF.colheaders{ii}) = extGRF.data(:,ii); 
    end
    clear extGRF
    %Trim and extract the force data to the kinematics times
    startTime = Storage('Jog05_rraKinematics.sto').getFirstTime();
    endTime = Storage('Jog05_rraKinematics.sto').getLastTime();
    startInd = find(GRFdata.time > startTime, 1) - 1;
    endInd = find(GRFdata.time > endTime, 1) - 1;
    %Set experimental GRF data variable
    %Structure is set as column 1 is the time
    %Structure is set as columns 2-4 are the right foot x,y,z values
    %Structure is set as columns 5-7 are the left foot x,y,z values
    Input.expGRF(:,1) = GRFdata.time(startInd:endInd);
    Input.expGRF(:,2) = GRFdata.calcn_r_ground_force_vx(startInd:endInd);
    Input.expGRF(:,3) = GRFdata.calcn_r_ground_force_vy(startInd:endInd);
    Input.expGRF(:,4) = GRFdata.calcn_r_ground_force_vz(startInd:endInd);
    Input.expGRF(:,5) = GRFdata.calcn_l_ground_force_vx(startInd:endInd);
    Input.expGRF(:,6) = GRFdata.calcn_l_ground_force_vy(startInd:endInd);
    Input.expGRF(:,7) = GRFdata.calcn_l_ground_force_vz(startInd:endInd);
    
    %Set the kinematics file to be used
    Input.kinematicsFile = [pwd,'\coordinates.sto'];

    %Set start and end times
    Input.startTime = startTime;
    Input.endTime = endTime;
    
    %Create and navigate to optimisation directory
    cd('..\..'); mkdir('FootContactOptimisations'); cd('FootContactOptimisations');
    
    %Set video object to record optimisation process
    Input.vidObj = VideoWriter('FootContactOptimisation_Progress.avi');
    
    figure; hold on
    
    
    %Run optimisation
    
    
    optimContactParameters(x0);
    
    
    %% Run inverse muscle-driven simulation of experimental data
    
    %Construct the MocoInverse tool.
    inverse = MocoInverse();

    %Construct a ModelProcessor and set it on the tool. The default
    %muscles in the model are replaced with optimization-friendly
    %DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    %parameters.
    modelProcessor = ModelProcessor([pwd,'\MocoScaledModel_StrengthScaled.osim']);
    cd('..\Gait');
    modelProcessor.append(ModOpAddExternalLoads([pwd,'\Jog05_grf.xml']));
    modelProcessor.append(ModOpIgnoreTendonCompliance());
    modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
    %Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
    %Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
    modelProcessor.append(ModOpAddReserves(1.0));
    inverse.setModel(modelProcessor);

    %Construct a TableProcessor of the coordinate data and pass it to the
    %inverse tool. TableProcessors can be used in the same way as
    %ModelProcessors by appending TableOperators to modify the base table.
    %A TableProcessor with no operators, as we have here, simply returns the
    %base table.
    
    %Before setting the table, the RRA kinematic file needs to be converted
    %to a states file that only has the joint kinematic values. This can be
    %done using a convenience function included with this repo.
    convertRRAtoStates('Jog05_rraKinematics.sto',rraModel,'coordinates.sto');    
   
    %Set the kinematics using the table processor
    inverse.setKinematics(TableProcessor([pwd,'\coordinates.sto']));

    %Initial time, final time, and mesh interval.
    %Initial and final time set based on coordinates data file
    %Mesh inverval arbitralily set at the moment - 0.01 is the lower end
    %value recommended in Moco documentation for gait simulations (maybe
    %not running though?)
    %%%% TO DO: could set this with better process\
        %%%%% NOTE: The Falisse et al. study uses 50 mesh intervals for a
        %%%%% half gait cycle, so this would be 100 for the full cycle in
        %%%%% this instance...
    inverse.set_initial_time(Storage('coordinates.sto').getFirstTime());
    inverse.set_final_time(Storage('coordinates.sto').getLastTime());
    inverse.set_mesh_interval(0.01);

    %By default, Moco gives an error if the kinematics contains extra columns.
    %Here, we tell Moco to allow (and ignore) those extra columns.
    inverse.set_kinematics_allow_extra_columns(true);
    
    %%%%%%% could adjust solution tolerance???

    %Solve the problem and write the solution to a Storage file.
    
    %Navigate to directory to store results
    cd('..\..'); mkdir('SimulationResults'); cd('SimulationResults');
    
    %Solve
    clc; inverseSolution = inverse.solve();
    inverseSolution.getMocoSolution().write('inverse_muscleDriven_solution.sto');
    
    %Cleanup default solution file
    delete MocoStudy_solution.sto
    
    %%%%%% paused after ~1 hour and 216 iterations...
    %%%%%% TO DO: finish simulation!

    
    %% Add contact elements to the feet of the model
    %%%%% TO DO: outline notes better
    %  1 under heel based on example 2D walking; one each under met1 and
    %  met 5. The locations for toe spheres will be placed at the same
    %  vertical level as the heel sphere, but the other location parameters
    %  dictated by the MET1 and MET5 markers in the model.
    
    %Navigate back to model directory
    cd('..\ExpData\Static');
    
    %Create new iteration of model
    contactModel = Model('MocoScaledModel_StrengthScaled.osim');
    
    %Add spheres using convenience function. This function also removes the
    %pelvis actuators from this model iteration.
    addSpheresToModel(contactModel,'MocoScaledModel_BaselineContact.osim');
    
    %% Run a test torque inverse simulation to see how the contact spheres
    %  work...
    
    %%%%%% matlab keeps crashing when using the contact model in this as
    %%%%%% it's existing object...try loading in as filename...? seems to
    %%%%%% work better, but didn't run the initial function first in this
    %%%%%% test...
    
    %%%% original tracking simulation having some issues with upward
    %%%% translation of model - but it seems this is related to the contact
    %%%% spheres being too stiff or their position being too close to the
    %%%% ground and generating heaps of force (hence the model 'bounces'
    %%%% up) - optimising these relative to the GRFs might fix this problem
    
    testTrack = MocoTrack();
    testTrack.setName('testSpheres_torqueTracking');
    
    % Construct a ModelProcessor and set it on the tool. The default
    % muscles in the model are replaced with optimization-friendly
    % DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    % parameters.
    modelProcessor = ModelProcessor('MocoScaledModel_BaselineContact.osim');
    modelProcessor.append(ModOpRemoveMuscles());
    % Add CoordinateActuators to the model degrees-of-freedom. This
    % ignores the pelvis coordinates which already have residual 
    % CoordinateActuators.
    modelProcessor.append(ModOpAddReserves(500));
    testTrack.setModel(modelProcessor);
    
    cd('..\Gait');
    testTrack.setStatesReference(TableProcessor([pwd,'\coordinates.sto']));
    testTrack.set_initial_time(Storage('coordinates.sto').getFirstTime());
    testTrack.set_final_time(Storage('coordinates.sto').getLastTime());
    testTrack.set_mesh_interval(0.01);
    cd('..\Static');
    
    testTrack.set_allow_unused_references(true);
    testTrack.set_track_reference_position_derivatives(true);
    
    study = testTrack.initialize();
    problem = study.updProblem();
    
    statesWeights = MocoWeightSet();
    model = modelProcessor.process();
    model.initSystem();
    %Get state variable names
    stateVars = model.getStateVariableNames();
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
    
    for ss = 1:length(jointStates)
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},10));        
    end
    clear ss
    
    
% % %     for cc = 0:cs.getSize()-1
% % %         statePath = cs.get(cc).getAbsolutePathString();
% % % % % %         if contains(char(statePath),'pelvis_ty')
% % % % % %             %Put a low weight on the tracking of the vertical pelvis
% % % % % %             %translation value (not speed) as this may not match well with
% % % % % %             %the addition of the contact spheres (i.e. the model may need
% % % % % %             %to move up/down to where it was depending on how this was
% % % % % %             %tracked, particularly during RRA).
% % % % % %             statesWeights.cloneAndAppend(MocoWeight([char(statePath),'/value'],1));
% % % % % %             statesWeights.cloneAndAppend(MocoWeight([char(statePath),'/speed'],10));
% % % % % %         else
% % %             statesWeights.cloneAndAppend(MocoWeight([char(statePath),'/value'],10));
% % %             statesWeights.cloneAndAppend(MocoWeight([char(statePath),'/speed'],10));
% % % % % %         end
% % %         clear statePath
% % %     end
% % %     clear cc
    statesGoal = MocoStateTrackingGoal.safeDownCast(problem.updGoal('state_tracking'));
    statesGoal.setWeightSet(statesWeights);
    
    %Put a large weight on the pelvis CoordinateActuators, which act as the
    %residual, or 'hand-of-god', forces which we would like to keep as small
    %as possible.
    effort = MocoControlGoal.safeDownCast(problem.updGoal("control_effort"));
    effort.setWeightForControl('/forceset/tau_pelvis_tilt', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_list', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_rotation', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_tx', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_ty', 10);
    effort.setWeightForControl('/forceset/tau_pelvis_tz', 10);

    %Add a contact force tracking goal, with the x and z positions of
    %spheres and their radii as parameters to optimise
    
    %Create goal
    contactGoal = MocoContactTrackingGoal('contact_goal',10);
    %Set the external loads
    cd('..\Gait'); extLoads = ExternalLoads('Jog05_grf.xml',true);
    extLoads.setDataFileName([pwd,'\Jog05_grf.mot']);    %update to include full path
    %Add external loads to goal
    contactGoal.setExternalLoads(extLoads);
    %Add the contact groups
    contactGroup_r = MocoContactTrackingGoalGroup();
    contactGroup_r.set_external_force_name('calcn_r');
    contactGroup_r.set_contact_force_paths(0,'/contactSphereHeel_r');
    contactGroup_r.set_contact_force_paths(1,'/contactSphereTarsals_r');
    contactGroup_r.set_contact_force_paths(2,'/contactSphereMetOneProx_r');
    contactGroup_r.set_contact_force_paths(3,'/contactSphereMetOneDist_r');
    contactGroup_r.set_contact_force_paths(4,'/contactSphereMetFiveProx_r');
    contactGroup_r.set_contact_force_paths(5,'/contactSphereMetFiveDist_r');
    contactGoal.addContactGroup(contactGroup_r);
    contactGroup_l = MocoContactTrackingGoalGroup();
    contactGroup_l.set_external_force_name('calcn_l');
    contactGroup_l.set_contact_force_paths(0,'/contactSphereHeel_l');
    contactGroup_l.set_contact_force_paths(1,'/contactSphereTarsals_l');
    contactGroup_l.set_contact_force_paths(2,'/contactSphereMetOneProx_l');
    contactGroup_l.set_contact_force_paths(3,'/contactSphereMetOneDist_l');
    contactGroup_l.set_contact_force_paths(4,'/contactSphereMetFiveProx_l');
    contactGroup_l.set_contact_force_paths(5,'/contactSphereMetFiveDist_l');
    contactGoal.addContactGroup(contactGroup_l);
    %Add goal to problem
    problem.addGoal(contactGoal);
    
    %Add contact sphere parameters to optimisation
    %Spheres are allowed to move a total of 0.1 in either direction
    %Sphere can be from 0.001 to 0.064 in size
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereHeel_r'));
    problem.addParameter(MocoParameter('contactSphereHeel_r_x',...  %parameter name
        '/contactSphereHeel_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereHeel_r_z',...  %parameter name
        '/contactSphereHeel_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereHeel_r_rad',...  %parameter name
        '/contactSphereHeel_r',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereTarsals_r'));
    problem.addParameter(MocoParameter('contactSphereTarsals_r_x',...  %parameter name
        '/contactSphereTarsals_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereTarsals_r_z',...  %parameter name
        '/contactSphereTarsals_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereTarsals_r_rad',...  %parameter name
        '/contactSphereTarsals_r',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereMetOneProx_r'));
    problem.addParameter(MocoParameter('contactSphereMetOneProx_r_x',...  %parameter name
        '/contactSphereMetOneProx_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetOneProx_r_z',...  %parameter name
        '/contactSphereMetOneProx_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetOneProx_r_rad',...  %parameter name
        '/contactSphereMetOneProx_r',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereMetOneDist_r'));
    problem.addParameter(MocoParameter('contactSphereMetOneDist_r_x',...  %parameter name
        '/contactSphereMetOneDist_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetOneDist_r_z',...  %parameter name
        '/contactSphereMetOneDist_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetOneDist_r_rad',...  %parameter name
        '/contactSphereMetOneDist_r',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereMetFiveProx_r'));
    problem.addParameter(MocoParameter('contactSphereMetFiveProx_r_x',...  %parameter name
        '/contactSphereMetFiveProx_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetFiveProx_r_z',...  %parameter name
        '/contactSphereMetFiveProx_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetFiveProx_r_rad',...  %parameter name
        '/contactSphereMetFiveProx_r',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereMetFiveDist_r'));
    problem.addParameter(MocoParameter('contactSphereMetFiveDist_r_x',...  %parameter name
        '/contactSphereMetFiveDist_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetFiveDist_r_z',...  %parameter name
        '/contactSphereMetFiveDist_r',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetFiveDist_r_rad',...  %parameter name
        '/contactSphereMetFiveDist_r',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereHeel_l'));
    problem.addParameter(MocoParameter('contactSphereHeel_l_x',...  %parameter name
        '/contactSphereHeel_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereHeel_l_z',...  %parameter name
        '/contactSphereHeel_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereHeel_l_rad',...  %parameter name
        '/contactSphereHeel_l',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereTarsals_l'));
    problem.addParameter(MocoParameter('contactSphereTarsals_l_x',...  %parameter name
        '/contactSphereTarsals_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereTarsals_l_z',...  %parameter name
        '/contactSphereTarsals_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereTarsals_l_rad',...  %parameter name
        '/contactSphereTarsals_l',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereMetOneProx_l'));
    problem.addParameter(MocoParameter('contactSphereMetOneProx_l_x',...  %parameter name
        '/contactSphereMetOneProx_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetOneProx_l_z',...  %parameter name
        '/contactSphereMetOneProx_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetOneProx_l_rad',...  %parameter name
        '/contactSphereMetOneProx_l',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereMetOneDist_l'));
    problem.addParameter(MocoParameter('contactSphereMetOneDist_l_x',...  %parameter name
        '/contactSphereMetOneDist_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetOneDist_l_z',...  %parameter name
        '/contactSphereMetOneDist_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetOneDist_l_rad',...  %parameter name
        '/contactSphereMetOneDist_l',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereMetFiveProx_l'));
    problem.addParameter(MocoParameter('contactSphereMetFiveProx_l_x',...  %parameter name
        '/contactSphereMetFiveProx_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetFiveProx_l_z',...  %parameter name
        '/contactSphereMetFiveProx_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetFiveProx_l_rad',...  %parameter name
        '/contactSphereMetFiveProx_l',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    sphere = SmoothSphereHalfSpaceForce.safeDownCast(model.getComponent('contactSphereMetFiveDist_l'));
    problem.addParameter(MocoParameter('contactSphereMetFiveDist_l_x',...  %parameter name
        '/contactSphereMetFiveDist_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(0)-0.1,sphere.get_contact_sphere_location().get(0)+0.1),...   %bounds on value
        0));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetFiveDist_l_z',...  %parameter name
        '/contactSphereMetFiveDist_l',... %path to component
        'contact_sphere_location',...      %property name
        MocoBounds(sphere.get_contact_sphere_location().get(2)-0.1,sphere.get_contact_sphere_location().get(2)+0.1),...   %bounds on value
        2));        %property element number
    problem.addParameter(MocoParameter('contactSphereMetFiveDist_l_rad',...  %parameter name
        '/contactSphereMetFiveDist_l',... %path to component
        'contact_sphere_radius',...      %property name
        MocoBounds(0.001,0.064)));
    
    %return to static directory
    cd('..\Static');
    
    %use tropter solver for parameter optimisation
    solver = study.initTropterSolver();
    
    %Solve and visualize.
    solution = study.solve();
% % %     study.visualize(solution);


    %%% with both contact goal and parameters added, solver doesn't want to
    %%% run due to shape of matrix?
        %%% try without parameters...it runs, parameters appear to be the
        %%% problem...maybe need to use tropter solver when using
        %%% parameters as recommended in Moco documentation?
    

    %Extract GRFs
    contactSpheres_r = StdVectorString();
    contactSpheres_l = StdVectorString();
    contactSpheres_r.add('contactSphereHeel_r');
    contactSpheres_r.add('contactSphereTarsals_r');
    contactSpheres_r.add('contactSphereMetOneProx_r');
    contactSpheres_r.add('contactSphereMetOneDist_r');
    contactSpheres_r.add('contactSphereMetFiveProx_r');
    contactSpheres_r.add('contactSphereMetFiveDist_r');
    contactSpheres_l.add('contactSphereHeel_l');
    contactSpheres_l.add('contactSphereTarsals_l');
    contactSpheres_l.add('contactSphereMetOneProx_l');
    contactSpheres_l.add('contactSphereMetOneDist_l');
    contactSpheres_l.add('contactSphereMetFiveProx_l');
    contactSpheres_l.add('contactSphereMetFiveDist_l');
    
    %Create a conventional ground reaction forces file by summing the contact
    %forces of contact spheres on each foot.
    %For details, navigate to
    %User Guide > Utilities > Model and trajectory utilities
    %in the Moco Documentation.
    externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(model, ...
                                 solution, contactSpheres_r, contactSpheres_l);
    opensimMoco.writeTableToFile(externalForcesTableFlat, ...
                                 'testSpheres_torqueTracking_solution_GRF.sto');
    

    
    
    %% Run a tracking solution that aims to track the experimental data
    %  coordinate states while also tracking the ground reaction forces
    %  using the contact spheres. Use this to optimise the spheres radii
    %  and location - and theoretically the end result should be a solution
    %  that accurately tracks kinematics and GRF data for use in subsequent
    %  predictive simulations.
    
    
    
    %%

end