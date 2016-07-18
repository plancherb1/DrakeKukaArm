%% matrix of distance of block from hand (base) and norm
function [ret,dRet] = distanceHandBlock(p,x,blockPosOffset,block_radius,magnitude_flag)
    [hand_pos, dHand_pos, ddHand_pos] = handPos(p,x);
    block_pos = x(blockPosOffset+1:blockPosOffset+3);
    dBlock_pos = zeros(3,p.num_positions);
    dBlock_pos(1,blockPosOffset+1) = 1;
    dBlock_pos(2,blockPosOffset+2) =1;
    dBlock_pos(3,blockPosOffset+3) = 1;
    xerr = hand_pos - block_pos;
    dXerr = [dHand_pos - dBlock_pos, zeros(3,p.num_velocities)];
    if magnitude_flag
        disSq = xerr'*eye(size(xerr,1))*xerr;
        dDisSq = 2*xerr'*dXerr;
        dis = sqrt(disSq) - block_radius;
        dDis = 1/2*disSq * dDisSq;
        ret = dis;
        dRet = dDis;
    else
        ret = xerr;
        dRet = dXerr;
    end
end