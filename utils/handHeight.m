%% function to return hand height
function [z,dZ] = handHeight(p,x)
    [hand_pos, dHand_pos] = handPos(p,x);
    z = hand_pos(3);
    dZ = [dHand_pos(3,:), zeros(1,p.num_velocities)];
end