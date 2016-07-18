%% function to return hand pos
function [hand_pos, dHand_pos, ddHand_pos] = handPos(p,x)
    q = x(1:p.num_positions);
    qd = x(p.num_positions+1:p.num_positions+p.num_velocities);
    kinsol_options.compute_gradients = true;
    kinsol = p.doKinematics(q,qd,kinsol_options);
    hand_body = p.findLinkId('iiwa_link_7');
    pos_on_hand_body = [0;0;0.25];
    [hand_pos, dHand_pos, ddHand_pos] = p.forwardKin(kinsol,hand_body,pos_on_hand_body);
end