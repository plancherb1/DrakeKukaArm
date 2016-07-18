%% function to return middle height
function [z,dZ] = middleHeight(p,x)
    kinsol = p.doKinematics(x(1:p.num_positions));
    hand_body = p.findLinkId('iiwa_link_4');
    pos_on_hand_body = [0;0;0];
    [hand_pos, dHand_pos] = p.forwardKin(kinsol,hand_body,pos_on_hand_body);
    z = hand_pos(3);
    dZ = [dHand_pos(3,:), zeros(1,p.num_velocities)];
end