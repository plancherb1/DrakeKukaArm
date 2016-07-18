%% function to compute kinematic trajectory
function [xtraj] = computeKinematicTraj(p,x0,xf,blockPosOffset)
    % helper function to do the inverse kinematics from q0 to qf as the
    % seed point and the constraints passed as Allcons
    function [xtraj] = computeIK(p,q0,qf,Allcons,T,N)
        % compute IK
        q_seed = qf;
        [q_end,snopt_info_ik,infeasible_constraint_ik] = inverseKin(p,q_seed,q0,Allcons{:});
        qtraj_guess = PPTrajectory(foh([0 T],[q0, q_end]));
        t_vec = linspace(0,T,N);

        % compute IK Traj
        [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(p,t_vec,qtraj_guess,qtraj_guess,Allcons{:});
        if snopt_info > 10
            error('IK fail snopt_info: %d\n', snopt_info);
        end
    end

    % set up problem per IRB planning example
    hand_id = findLinkId(p,'iiwa_link_7');
    block_id = findLinkId(p,'block_actual');
    pt_on_hand = [0,0,0.25]';
    pt_on_block = [0,0,0]';
    q0 = x0(1:p.num_positions);
    qf = xf(1:p.num_positions);
    qf_block = qf(blockPosOffset+1:size(qf,1));
    hand_xyz_xf = handPos(p,xf);
    Tf = 1;
    n = 10;

    % solve for x0 to xf
    tol = 0.001;
    ikCons{1,1} = WorldPositionConstraint(p,hand_id,pt_on_hand,hand_xyz_xf-tol,hand_xyz_xf+tol,[Tf,Tf]);
    ikCons{2,1} = WorldPositionConstraint(p,block_id,pt_on_block,qf_block-tol,qf_block+tol,[Tf,Tf]);
    [xtraj] = computeIK(p,q0,qf,ikCons,Tf,n);
end