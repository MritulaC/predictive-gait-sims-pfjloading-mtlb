function addSpheresToModel(contactModel,outputModelName)

    %Inputs
    %
    % rraFile       .sto file containing RRA kinematics. These kinematics
    %               should only have the coordinate name (i.e. not the
    %               absolute path string to the coordinate)
    %
    % outputModelName   optional filename to include for the output (will
    %                   default to contactModel.osim)
    
    import org.opensim.modeling.*
    
    if nargin < 1
       error('At least 1 input of model object is needed to add spheres to.') 
    end
    if nargin < 2
        outputModelName = 'contactModel.osim';
    end
    
    %Set starting generic positions. These are based on visual inspection
    %of the model and placement in the same positions as outlined in the
    %generic model in Falisse et al. (2019).
    %NOTE: these values are likely only correct for the currently scaled
    %experimental model - it is probably better to locate these same
    %positions in the generic model (perhaps using markers) - and then
    %these will transfer across to the scaled model.
    heelPos_r = Vec3(0.0157484,0.010435842527310599,-0.0109827);
    tarsalsPos_r = Vec3(0.0753592,0.010435842527310599,0.0226294);
    metOneProxPos_r = Vec3(0.184621,0.010435842527310599,-0.0278901);
    metOneDistPos_r = Vec3(0.254175,0.010435842527310599,-0.0166661);
    metFiveProxPos_r = Vec3(0.140891,0.010435842527310599,0.0463663);
    metFiveDistPos_r = Vec3(0.198869,0.010435842527310599,0.0629414);
    heelPos_l = Vec3(0.0157484,0.010435842527310599,0.0109827);
    tarsalsPos_l = Vec3(0.0753592,0.010435842527310599,-0.0226294);
    metOneProxPos_l = Vec3(0.184621,0.010435842527310599,0.0278901);
    metOneDistPos_l = Vec3(0.254175,0.010435842527310599,0.0166661);
    metFiveProxPos_l = Vec3(0.140891,0.010435842527310599,-0.0463663);
    metFiveDistPos_l = Vec3(0.198869,0.010435842527310599,-0.0629414);
    
    %Create spheres
    contactHeel_r = SmoothSphereHalfSpaceForce('contactSphereHeel_r',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_r')),...     %sphere socket frame
        heelPos_r,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation    
    contactTarsals_r = SmoothSphereHalfSpaceForce('contactSphereTarsals_r',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_r')),...     %sphere socket frame
        tarsalsPos_r,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation
    contactMetOneProx_r = SmoothSphereHalfSpaceForce('contactSphereMetOneProx_r',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_r')),...     %sphere socket frame
        metOneProxPos_r,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation
    contactMetOneDist_r = SmoothSphereHalfSpaceForce('contactSphereMetOneDist_r',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_r')),...     %sphere socket frame
        metOneDistPos_r,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation
    contactMetFiveProx_r = SmoothSphereHalfSpaceForce('contactSphereMetFiveProx_r',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_r')),...     %sphere socket frame
        metFiveProxPos_r,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation
    contactMetFiveDist_r = SmoothSphereHalfSpaceForce('contactSphereMetFiveDist_r',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_r')),...     %sphere socket frame
        metFiveDistPos_r,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation
    contactHeel_l = SmoothSphereHalfSpaceForce('contactSphereHeel_l',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_l')),...     %sphere socket frame
        heelPos_l,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation    
    contactTarsals_l = SmoothSphereHalfSpaceForce('contactSphereTarsals_l',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_l')),...     %sphere socket frame
        tarsalsPos_l,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation
    contactMetOneProx_l = SmoothSphereHalfSpaceForce('contactSphereMetOneProx_l',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_l')),...     %sphere socket frame
        metOneProxPos_l,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation
    contactMetOneDist_l = SmoothSphereHalfSpaceForce('contactSphereMetOneDist_l',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_l')),...     %sphere socket frame
        metOneDistPos_l,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation
    contactMetFiveProx_l = SmoothSphereHalfSpaceForce('contactSphereMetFiveProx_l',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_l')),...     %sphere socket frame
        metFiveProxPos_l,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation
    contactMetFiveDist_l = SmoothSphereHalfSpaceForce('contactSphereMetFiveDist_l',...    %object name
        Frame.safeDownCast(contactModel.getBodySet().get('calcn_l')),...     %sphere socket frame
        metFiveDistPos_l,...     %sphere location
        0.032,...       %radius
        Frame.safeDownCast(contactModel.getGround()),...     %socket half space frame
        Vec3(0,0,0),...     %socket half space location
        Vec3(0,0,-1.5707963267948966));     %half space orientation
        
    %Set remaining parameters. These parameters also come from the Falisse
    %et al. (2019) paper/code. Falisse uses 1000000 for stiffness though
    %Stiffness here comes from Umberger 2D walking Moco example
    contactHeel_r.set_stiffness(3067776);contactHeel_r.set_dissipation(2);
    contactHeel_r.set_static_friction(0.8); contactHeel_r.set_dynamic_friction(0.8);
    contactHeel_r.set_viscous_friction(0.5); contactHeel_r.set_transition_velocity(0.2);
    contactHeel_r.set_hertz_smoothing(300); contactHeel_r.set_hunt_crossley_smoothing(50);
    contactHeel_r.set_dissipation(2);
    
    contactTarsals_r.set_stiffness(3067776); contactTarsals_r.set_dissipation(2);
    contactTarsals_r.set_static_friction(0.8); contactTarsals_r.set_dynamic_friction(0.8);
    contactTarsals_r.set_viscous_friction(0.5); contactTarsals_r.set_transition_velocity(0.2);
    contactTarsals_r.set_hertz_smoothing(300); contactTarsals_r.set_hunt_crossley_smoothing(50);
    contactTarsals_r.set_dissipation(2);
    
    contactMetOneProx_r.set_stiffness(3067776); contactMetOneProx_r.set_dissipation(2);
    contactMetOneProx_r.set_static_friction(0.8); contactMetOneProx_r.set_dynamic_friction(0.8);
    contactMetOneProx_r.set_viscous_friction(0.5); contactMetOneProx_r.set_transition_velocity(0.2);
    contactMetOneProx_r.set_hertz_smoothing(300); contactMetOneProx_r.set_hunt_crossley_smoothing(50);
    contactMetOneProx_r.set_dissipation(2);
    
    contactMetOneDist_r.set_stiffness(3067776); contactMetOneDist_r.set_dissipation(2);
    contactMetOneDist_r.set_static_friction(0.8); contactMetOneDist_r.set_dynamic_friction(0.8);
    contactMetOneDist_r.set_viscous_friction(0.5); contactMetOneDist_r.set_transition_velocity(0.2);
    contactMetOneDist_r.set_hertz_smoothing(300); contactMetOneDist_r.set_hunt_crossley_smoothing(50);
    contactMetOneDist_r.set_dissipation(2);
    
    contactMetFiveProx_r.set_stiffness(3067776); contactMetFiveProx_r.set_dissipation(2);
    contactMetFiveProx_r.set_static_friction(0.8); contactMetFiveProx_r.set_dynamic_friction(0.8);
    contactMetFiveProx_r.set_viscous_friction(0.5); contactMetFiveProx_r.set_transition_velocity(0.2);
    contactMetFiveProx_r.set_hertz_smoothing(300); contactMetFiveProx_r.set_hunt_crossley_smoothing(50);
    contactMetFiveProx_r.set_dissipation(2);
    
    contactMetFiveDist_r.set_stiffness(3067776); contactMetFiveDist_r.set_dissipation(2);
    contactMetFiveDist_r.set_static_friction(0.8); contactMetFiveDist_r.set_dynamic_friction(0.8);
    contactMetFiveDist_r.set_viscous_friction(0.5); contactMetFiveDist_r.set_transition_velocity(0.2);
    contactMetFiveDist_r.set_hertz_smoothing(300); contactMetFiveDist_r.set_hunt_crossley_smoothing(50);
    contactMetFiveDist_r.set_dissipation(2);
    
    contactHeel_l.set_stiffness(3067776); contactHeel_l.set_dissipation(2);
    contactHeel_l.set_static_friction(0.8); contactHeel_l.set_dynamic_friction(0.8);
    contactHeel_l.set_viscous_friction(0.5); contactHeel_l.set_transition_velocity(0.2);
    contactHeel_l.set_hertz_smoothing(300); contactHeel_l.set_hunt_crossley_smoothing(50);
    contactHeel_l.set_dissipation(2);
    
    contactTarsals_l.set_stiffness(3067776); contactTarsals_l.set_dissipation(2);
    contactTarsals_l.set_static_friction(0.8); contactTarsals_l.set_dynamic_friction(0.8);
    contactTarsals_l.set_viscous_friction(0.5); contactTarsals_l.set_transition_velocity(0.2);
    contactTarsals_l.set_hertz_smoothing(300); contactTarsals_l.set_hunt_crossley_smoothing(50);
    contactTarsals_l.set_dissipation(2);
    
    contactMetOneProx_l.set_stiffness(3067776); contactMetOneProx_l.set_dissipation(2);
    contactMetOneProx_l.set_static_friction(0.8); contactMetOneProx_l.set_dynamic_friction(0.8);
    contactMetOneProx_l.set_viscous_friction(0.5); contactMetOneProx_l.set_transition_velocity(0.2);
    contactMetOneProx_l.set_hertz_smoothing(300); contactMetOneProx_l.set_hunt_crossley_smoothing(50);
    contactMetOneProx_l.set_dissipation(2);
    
    contactMetOneDist_l.set_stiffness(3067776); contactMetOneDist_l.set_dissipation(2);
    contactMetOneDist_l.set_static_friction(0.8); contactMetOneDist_l.set_dynamic_friction(0.8);
    contactMetOneDist_l.set_viscous_friction(0.5); contactMetOneDist_l.set_transition_velocity(0.2);
    contactMetOneDist_l.set_hertz_smoothing(300); contactMetOneDist_l.set_hunt_crossley_smoothing(50);
    contactMetOneDist_l.set_dissipation(2);
    
    contactMetFiveProx_l.set_stiffness(3067776); contactMetFiveProx_l.set_dissipation(2);
    contactMetFiveProx_l.set_static_friction(0.8); contactMetFiveProx_l.set_dynamic_friction(0.8);
    contactMetFiveProx_l.set_viscous_friction(0.5); contactMetFiveProx_l.set_transition_velocity(0.2);
    contactMetFiveProx_l.set_hertz_smoothing(300); contactMetFiveProx_l.set_hunt_crossley_smoothing(50);
    contactMetFiveProx_l.set_dissipation(2);
    
    contactMetFiveDist_l.set_stiffness(3067776); contactMetFiveDist_l.set_dissipation(2);
    contactMetFiveDist_l.set_static_friction(0.8); contactMetFiveDist_l.set_dynamic_friction(0.8);
    contactMetFiveDist_l.set_viscous_friction(0.5); contactMetFiveDist_l.set_transition_velocity(0.2);
    contactMetFiveDist_l.set_hertz_smoothing(300); contactMetFiveDist_l.set_hunt_crossley_smoothing(50);
    contactMetFiveDist_l.set_dissipation(2);
    
    
    %Connect to model
    contactModel.addComponent(Component.safeDownCast(contactHeel_r));
    contactModel.addComponent(Component.safeDownCast(contactTarsals_r));
    contactModel.addComponent(Component.safeDownCast(contactMetOneProx_r));
    contactModel.addComponent(Component.safeDownCast(contactMetOneDist_r));
    contactModel.addComponent(Component.safeDownCast(contactMetFiveProx_r));
    contactModel.addComponent(Component.safeDownCast(contactMetFiveDist_r));
    contactModel.addComponent(Component.safeDownCast(contactHeel_l));
    contactModel.addComponent(Component.safeDownCast(contactTarsals_l));
    contactModel.addComponent(Component.safeDownCast(contactMetOneProx_l));
    contactModel.addComponent(Component.safeDownCast(contactMetOneDist_l));
    contactModel.addComponent(Component.safeDownCast(contactMetFiveProx_l));
    contactModel.addComponent(Component.safeDownCast(contactMetFiveDist_l));
    
% % %     %Remove pelvis actuators from model due to contact supporting this now
% % %     
% % %     %Clone original force set
% % %     clonedForceSet = contactModel.updForceSet().clone();
% % %     
% % %     %Remove forces from the cloned set
% % %     clonedForceSet.remove(clonedForceSet.get('tau_pelvis_tilt'));
% % %     clonedForceSet.remove(clonedForceSet.get('tau_pelvis_list'));
% % %     clonedForceSet.remove(clonedForceSet.get('tau_pelvis_rotation'));
% % %     clonedForceSet.remove(clonedForceSet.get('tau_pelvis_tx'));
% % %     clonedForceSet.remove(clonedForceSet.get('tau_pelvis_ty'));
% % %     clonedForceSet.remove(clonedForceSet.get('tau_pelvis_tz'));
% % %     
% % %     %Clear force set from original model
% % %     contactModel.getForceSet().clearAndDestroy();
% % %     
% % %     %Clone and append the updated force set
% % %     for ii = 0:clonedForceSet.getSize()-1
% % %         contactModel.updForceSet().cloneAndAppend(clonedForceSet.get(ii));
% % %     end
% % %     clear ii

    %Instead of removing pelvis actuators, set optimal force to a really
    %low value. Combined with a high tracking weight on the controller,
    %this should minimise their use. It seems they may still be needed in
    %the problem to avoid conflicts with the model trajectory and problem.
    %If they are removed then a reserve actuator gets added to the
    %coordinate, which is also undesirable.
    actu = CoordinateActuator.safeDownCast(contactModel.updForceSet().get('tau_pelvis_tilt'));
    actu.setOptimalForce(1e-10);  actu.setMaxControl(1); actu.setMinControl(-1);
    actu = CoordinateActuator.safeDownCast(contactModel.updForceSet().get('tau_pelvis_list'));
    actu.setOptimalForce(1e-10);  actu.setMaxControl(1); actu.setMinControl(-1);
    actu = CoordinateActuator.safeDownCast(contactModel.updForceSet().get('tau_pelvis_rotation'));
    actu.setOptimalForce(1e-10);  actu.setMaxControl(1); actu.setMinControl(-1);
    actu = CoordinateActuator.safeDownCast(contactModel.updForceSet().get('tau_pelvis_tx'));
    actu.setOptimalForce(1e-10);  actu.setMaxControl(1); actu.setMinControl(-1);
    actu = CoordinateActuator.safeDownCast(contactModel.updForceSet().get('tau_pelvis_ty'));
    actu.setOptimalForce(1e-10);  actu.setMaxControl(1); actu.setMinControl(-1);
    actu = CoordinateActuator.safeDownCast(contactModel.updForceSet().get('tau_pelvis_tz'));
    actu.setOptimalForce(1e-10);  actu.setMaxControl(1); actu.setMinControl(-1);
    
% % %     contactModel.updForceSet().get('tau_pelvis_tilt').set_appliesForce(false);
% % %     contactModel.updForceSet().get('tau_pelvis_list').set_appliesForce(false);
% % %     contactModel.updForceSet().get('tau_pelvis_rotation').set_appliesForce(false);
% % %     contactModel.updForceSet().get('tau_pelvis_tx').set_appliesForce(false);
% % %     contactModel.updForceSet().get('tau_pelvis_ty').set_appliesForce(false);
% % %     contactModel.updForceSet().get('tau_pelvis_tz').set_appliesForce(false);

    %Output model
    contactModel.finalizeConnections();
    contactModel.print(outputModelName);
    
end