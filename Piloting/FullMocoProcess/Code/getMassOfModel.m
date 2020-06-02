function totalMass = getMassOfModel(osimModel)
    totalMass = 0;
    allBodies = osimModel.getBodySet();
    for i=0:allBodies.getSize()-1
        curBody = allBodies.get(i);
        totalMass = totalMass + curBody.getMass();
    end
end