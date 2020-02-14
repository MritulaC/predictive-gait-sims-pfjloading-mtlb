%Note that this function comes with those packaged with the Rajagopal et al. (2015) gait model

function osimModel_scaledForces = scaleOptimalForceSubjectSpecific(osimModel_generic, osimModel_scaled, height_generic, height_scaled)
    mass_generic = getMassOfModel(osimModel_generic);
    mass_scaled = getMassOfModel(osimModel_scaled);
    
    Vtotal_generic = 47.05 * mass_generic * height_generic + 1289.6;
    Vtotal_scaled = 47.05 * mass_scaled * height_scaled + 1289.6;
    
    allMuscles_generic = osimModel_generic.getMuscles();
    allMuscles_scaled = osimModel_scaled.getMuscles();
    
    for i=0:allMuscles_generic.getSize()-1
        currentMuscle_generic = allMuscles_generic.get(i);
        currentMuscle_scaled = allMuscles_scaled.get(i);
        
        lmo_generic = currentMuscle_generic.getOptimalFiberLength();
        lmo_scaled = currentMuscle_scaled.getOptimalFiberLength();
        
        forceScaleFactor = (Vtotal_scaled/Vtotal_generic)/(lmo_scaled/lmo_generic);
        
        currentMuscle_scaled.setMaxIsometricForce( forceScaleFactor * currentMuscle_generic.getMaxIsometricForce() );
    end
    
    osimModel_scaledForces = osimModel_scaled;    
end