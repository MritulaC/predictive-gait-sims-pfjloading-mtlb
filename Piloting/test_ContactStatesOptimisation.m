%% Import opensim libraries and set-up

import org.opensim.modeling.*
warning off

%Set main directory
mainDir = pwd;

%Add supplementary code folder to path
addpath(genpath('..\Code\Supplementary'));

%Add model geometry directory
ModelVisualizer.addDirToGeometrySearchPaths('..\GenericModel\Geometry');

%% Set-up a torque driven inverse problem

%Construct the MocoInverse tool.
inverse = MocoInverse();
inverse.setName('gaitInverse_torqueDriven');

%Get the scaled muscle model to use in the inverse simulations
inverseModel = Model('..\ExpData\Static\MocoScaledModel_StrengthScaled.osim');

%Clear the forceset from the model (includes contact spheres)
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

%Set the model in inverse study
modelProcessor = ModelProcessor(inverseModel);                                  %Add the OpenSim model
modelProcessor.append(ModOpAddExternalLoads([pwd,'\..\ExpData\Gait\Jog05_grf.xml']));   %Add the external loads
inverse.setModel(modelProcessor); 

%Construct a TableProcessor of the coordinate data and pass it to the
%inverse tool
inverse.setKinematics(TableProcessor([pwd,'\..\ExpData\Gait\coordinates.sto']));

%Allow extra columns in kinemtics file just in case
inverse.set_kinematics_allow_extra_columns(true);

%Load the coordinates and identify the starting time. This corresponds with
%just before a right foot strike.
coordinatesData = Storage('..\ExpData\Gait\coordinates.sto');
coordinatesStartTime = coordinatesData.getFirstTime();

%Load the GRF data
grfTable = TimeSeriesTable('..\ExpData\Gait\Jog05_grf.mot');
grfData = osimTableToStruct(grfTable);

%Create a logical for where the right & left foot is in contact with the ground
%based on 50N threshold (which is what was originally used)
rightGrfOn = logical(grfData.calcn_r_ground_force_vy > 50);
leftGrfOn = logical(grfData.calcn_l_ground_force_vy > 50);
%Find the index that corresponds to the foot strike associated with the
%kinematic data
grfStartSearch = find(grfData.time > coordinatesStartTime,1);
grfStartInd = find(rightGrfOn(grfStartSearch:end),1) + (grfStartSearch-2);

%Find the next left foot strike following the starting point to indicate a
%half gait cycle
grfEndInd = find(leftGrfOn(grfStartInd:end),1) + (grfStartInd-2);

%Identify the start and stop times in the grf time index
startTime = grfData.time(grfStartInd);
endTime = grfData.time(grfEndInd); 

%Set timing details
inverse.set_initial_time(startTime);
inverse.set_final_time(endTime);
inverse.set_mesh_interval((endTime-startTime) / 50); %50 mesh interval?

%Save problem to text to review if necessary
inverse.print('gaitInverse_torqueDriven.omoco'); 

%Solve problem
clc
inverseSolution_torqueDriven = inverse.solve();

%Rename the moco solution that comes from the inverse problem
movefile('MocoStudy_solution.sto','gaitInverse_torqueDriven.sto');

%Extract the inveser solution as a moco solution to grab all aspects of the
%solution not usually output (e.g. kinematics)
inverseSolution_torqueDriven.getMocoSolution().write('gaitInverse_torqueDriven_MocoSolution.sto');

%Set as accesible object
inverseSolution_torqueDriven_Moco = inverseSolution_torqueDriven.getMocoSolution();

%% Attempt a state and contact tracking torque driven problem

%Set the global tracking weights for each aspect of the problem
controlEffortWeight = 0.5;
stateTrackingWeight = 1;
GRFTrackingWeight   = 10;

%Define the motion tracking problem
track = MocoTrack();
track.setName('gaitTracking_torqueDriven');

%Set the kinematics
tableStatesProcessor = TableProcessor([pwd,'\..\ExpData\Gait\coordinates.sto']);
track.set_allow_unused_references(true);

%Set the model
%As with earlier, we want to remove the muscles from the forceset, but keep
%the contact spheres
%Can do this by copying over the sphere forces to a blank forceset using
%two models
baseModel = Model('..\ExpData\Static\MocoScaledModel_StrengthScaled.osim');
trackModel = Model('..\ExpData\Static\MocoScaledModel_StrengthScaled.osim');

%Clear force set from track model
trackModel.updForceSet().clearAndDestroy();

%Loop through base model force set and copy over the sphere forces
for ff = 0:baseModel.updForceSet().getSize()-1
    if contains(char(baseModel.updForceSet().get(ff).getName()),'contact')
        %Copy over to track model force set
        trackModel.updForceSet().cloneAndAppend(baseModel.updForceSet().get(ff));
    end    
end
clear ff

%Loop through and add coordinate actuators back into the models forceset to
%drive the motion.
for cc = 0:trackModel.getCoordinateSet().getSize()-1
    
    %Create the coordinate actuator
    actu = CoordinateActuator();
    
    %Get name of the current coordinate
    currCoord = trackModel.getCoordinateSet().get(cc).getName();
    
    %Set actuator coordinate
    actu.setCoordinate(trackModel.getCoordinateSet().get(cc));
    
    %Set actuator name
    actu.setName(['tau_',char(currCoord)]);
    
    %Set optimal force
    actu.setOptimalForce(1000);
    
    %Set control levels    
    actu.setMaxControl(inf);
    actu.setMinControl(-inf);
    
    %Add actuator to model
    trackModel.updForceSet().cloneAndAppend(actu);
    
end
clear cc

%Finalise model connections
trackModel.finalizeConnections();

%Set model in problem
modelProcessor = ModelProcessor(trackModel);
track.setModel(modelProcessor);  

%Set times in tracking problem
track.set_initial_time(startTime);
track.set_final_time(endTime);

%Apply states to tracking problem
track.setStatesReference(tableStatesProcessor);
track.set_states_global_tracking_weight(stateTrackingWeight);

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
%Note that this doesn't actually change it from he default, but would if
%the states tracking weight was changed above
%Let pelvis translations change more than other kinematics by dropping
%their weight down
for ss = 1:length(jointStates)
    if contains(jointStates{ss},'pelvis_tx') || ...
            contains(jointStates{ss},'pelvis_ty') || ...
            contains(jointStates{ss},'pelvis_tz')
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},stateTrackingWeight/10));        
    else
        statesWeights.cloneAndAppend(MocoWeight(jointStates{ss,1},stateTrackingWeight));        
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
effort.setWeight(controlEffortWeight);
effort.setExponent(2);
effort.setDivideByDisplacement(true);

%Add a contact tracking weight if a GRF weight is set
if GRFTrackingWeight ~= 0
    
    %Create the GRF tracking goal
    contactTracking = MocoContactTrackingGoal('contact',GRFTrackingWeight);
    
    %Set the external loads file
    contactTracking.setExternalLoadsFile([pwd,'\..\ExpData\Gait\Jog05_grf.xml']);
    
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
    problem.addGoal(contactTracking);
    
end

%Set state bounds in the problem
problem.setStateInfo('/jointset/ground_pelvis/pelvis_list/value', [-10*pi/180, 10*pi/180]);
problem.setStateInfo('/jointset/ground_pelvis/pelvis_tilt/value', [-30*pi/180, 30*pi/180]);
problem.setStateInfo('/jointset/ground_pelvis/pelvis_rotation/value', [-15*pi/180, 15*pi/180]);
problem.setStateInfo('/jointset/ground_pelvis/pelvis_tx/value', [0, 5]);
problem.setStateInfo('/jointset/ground_pelvis/pelvis_ty/value', [0.75, 1.25]);
problem.setStateInfo('/jointset/ground_pelvis/pelvis_tz/value', [0, 1]);
problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-20*pi/180, 60*pi/180]);
problem.setStateInfo('/jointset/hip_r/hip_adduction_r/value', [-20*pi/180, 20*pi/180]);
problem.setStateInfo('/jointset/hip_r/hip_rotation_r/value', [-20*pi/180, 20*pi/180]);
problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', [-20*pi/180, 60*pi/180]);
problem.setStateInfo('/jointset/hip_l/hip_adduction_l/value', [-20*pi/180, 20*pi/180]);
problem.setStateInfo('/jointset/hip_l/hip_rotation_l/value', [-20*pi/180, 20*pi/180]);
problem.setStateInfo('/jointset/walker_knee_r/knee_angle_r/value', [0*pi/180, 120*pi/180]);
problem.setStateInfo('/jointset/walker_knee_l/knee_angle_l/value', [0*pi/180, 120*pi/180]);
problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [-40*pi/180, 30*pi/180]);
problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', [-40*pi/180, 30*pi/180]);
problem.setStateInfo('/jointset/back/lumbar_extension/value', [-10*pi/180, 20*pi/180]);
problem.setStateInfo('/jointset/back/lumbar_bending/value', [-15*pi/180, 15*pi/180]);
problem.setStateInfo('/jointset/back/lumbar_rotation/value', [-30*pi/180, 30*pi/180]);

%Access the CasADi solver to set the mesh interval
solver = study.initCasADiSolver();

%Update mesh interval to desired level
nMeshIntervals = 50;
solver.set_num_mesh_intervals(nMeshIntervals);

% % % % % NOTE: won't set the guess from inverse solution in this yet...

%Set number of iterations for solver
solver.set_optim_max_iterations(1000);

%Save problem to text to review if necessary
study.print('gaitTracking_torqueDriven.omoco');
    
%Solve problem
clc
trackingSolution = study.solve();

%%%%% This doesn't work well at all - the solver doesn't succeed after 1000
%%%%% iterations, and the motion doesn't look right, so there's no point of
%%%%% looking at the GRFs.
%%%%%
%%%%% It may be better to start from scratch with a generic model and
%%%%% marker data and manage the outputs from the early stage to be
%%%%% suitable for Moco.

%%








