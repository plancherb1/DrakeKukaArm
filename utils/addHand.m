%% add a urdf as hand to the plant
function [p] = addHand(p,urdfPath)
    options_hand.weld_to_link = p.findLinkId('iiwa_link_7');
    options_hand.axis = [0;0;1];
    p = p.addRobotFromURDF(getFullPathFromRelativePath(urdfPath),[0;0;.05],[pi/2;0;0],options_hand);
    p = p.compile();
end