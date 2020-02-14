function err = optimContactParameters(x)
    
    %Function to use with fminsearchbnd to optimise the contact parameters
    %attached to the feet of the model. The parameters of x and z location,
    %plus sphere radii are altered until they match the three-dimensional
    %ground reaction forces.
    
    import org.opensim.modeling.*
    
    %Grab global variables
    global Input
    global optimModel
    
    %% Set the contact parameters in model to current iteration
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereHeel_r'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(1)); currLocation.set(2,x(2));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(25));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereTarsals_r'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(3)); currLocation.set(2,x(4));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(26));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereMetOneProx_r'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(5)); currLocation.set(2,x(6));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(27));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereMetOneDist_r'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(7)); currLocation.set(2,x(8));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(28));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereMetFiveProx_r'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(9)); currLocation.set(2,x(10));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(29));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereMetFiveDist_r'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(11)); currLocation.set(2,x(12));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(30));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereHeel_l'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(13)); currLocation.set(2,x(14));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(31));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereTarsals_l'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(15)); currLocation.set(2,x(16));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(32));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereMetOneProx_l'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(17)); currLocation.set(2,x(18));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(33));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereMetOneDist_l'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(19)); currLocation.set(2,x(20));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(34));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereMetFiveProx_l'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(21)); currLocation.set(2,x(22));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(35));
    
    currSphere = SmoothSphereHalfSpaceForce.safeDownCast(optimModel.getComponent('contactSphereMetFiveDist_l'));
    currLocation = currSphere.get_contact_sphere_location();
    currLocation.set(0,x(23)); currLocation.set(2,x(24));
    currSphere.set_contact_sphere_location(currLocation);
    currSphere.set_contact_sphere_radius(x(36));
    
    %Finalise model connections
    optimModel.finalizeConnections();
    
    %% Run a torque-drive inverse simulation to extract contact forces
    
    %Construct the MocoInverse tool.
    inverse = MocoInverse();
    
    % Construct a ModelProcessor and set it on the tool.
    modelProcessor = ModelProcessor(optimModel);
    % Add CoordinateActuators to the model degrees-of-freedom. This
    % ignores the pelvis coordinates which already have residual 
    % CoordinateActuators.
    modelProcessor.append(ModOpAddReserves(500));
    inverse.setModel(modelProcessor);

    %Set parameters for inverse tool
    inverse.setKinematics(TableProcessor(Input.kinematicsFile));
    inverse.set_initial_time(Storage(Input.kinematicsFile).getFirstTime());
    inverse.set_final_time(Storage(Input.kinematicsFile).getLastTime());
    inverse.set_mesh_interval(0.01);
    inverse.set_kinematics_allow_extra_columns(true);
    
    %Solve
    clc; inverseSolution = inverse.solve();
    
    %Get full solution
    fullSolution = inverseSolution.getMocoSolution();
    
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
    
    %Place ground reaction forces in a table format
    externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(optimModel, ...
                                 fullSolution, contactSpheres_r, contactSpheres_l);
    
	%Extract and plot GRFs
    
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
    
    %Plot result iteration
    
    %Right foot
    subplot(3,2,1); cla; hold on
    plot(Input.expGRF(:,1),Input.expGRF(:,2),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,1),'r--','LineWidth',1);
    subplot(3,2,3); cla; hold on
    plot(Input.expGRF(:,1),Input.expGRF(:,3),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,2),'r--','LineWidth',1);
    subplot(3,2,5); cla; hold on
    plot(Input.expGRF(:,1),Input.expGRF(:,4),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,3),'r--','LineWidth',1);
    
    %Left foot
    subplot(3,2,2); cla; hold on
    plot(Input.expGRF(:,1),Input.expGRF(:,5),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,7),'r--','LineWidth',1);
    subplot(3,2,4); cla; hold on
    plot(Input.expGRF(:,1),Input.expGRF(:,6),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,8),'r--','LineWidth',1);
    subplot(3,2,6); cla; hold on
    plot(Input.expGRF(:,1),Input.expGRF(:,7),'k','LineWidth',1.5)
    plot(time(:,1),grfData(:,9),'r--','LineWidth',1);
    
    %Calculate error
    

    %remove inverse solver file                         
	delete MocoStudy_solution.sto
                             
    
    
    
end