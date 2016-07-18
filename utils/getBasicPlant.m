%% get the basic plant
function [p] = getBasicPlant()
    options.floating = false;
    options.terrain = RigidBodyFlatTerrain();
    w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
    p = RigidBodyManipulator('urdf/iiwa14.urdf',options);
    warning(w);
end