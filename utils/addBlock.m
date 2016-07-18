%% add the block to the combined plant
function [p] = addBlock(p,urdfPath)
    options_block.floating = false;
    p = p.addRobotFromURDF(getFullPathFromRelativePath(urdfPath),[0;0;0],[0;0;0],options_block);
    p = p.compile();
end