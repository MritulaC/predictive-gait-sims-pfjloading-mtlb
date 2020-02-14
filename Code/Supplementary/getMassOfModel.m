%Note that this function comes with those packaged with the Rajagopal et al. (2015) gait model

function totalMass = getMassOfModel(osimModel)
    totalMass = 0;
    allBodies = osimModel.getBodySet();
    for i=0:allBodies.getSize()-1
        curBody = allBodies.get(i);
        totalMass = totalMass + curBody.getMass();
    end
end