%% function to return hand vel
function [hand_vel,dHand_vel] = handVel(p,x)
    [hand_pos, dHand_pos, ddHand_pos] = handPos(p,x);
    qd = x(p.num_positions+1:p.num_positions+p.num_velocities);
    hand_vel = dHand_pos * qd;
    dHand_vel = [repmult(ddHand_pos,qd),dHand_pos];
end