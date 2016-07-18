%% controlled fall to block followed by raising it up
function runMoveBlock()
    
    % get the plant
    p = getBasicPlant();
        
    % add the block reprsenting the hand
    p = addHand(p,'../Atlas/urdf/robotiq_box.urdf');
    n_arm_pos = p.num_positions;
    n_arm_vel = p.num_velocities;
    n_arm_u = size(p.umin,1);
    
    % add the actuated block
    p = addBlock(p,'urdf/XYZActuatedBlock.urdf');
    n_block_pos = p.num_positions - n_arm_pos;
    n_block_vel = p.num_velocities - n_arm_vel;
    n_block_u = size(p.umin,1) - n_arm_u;
    
    % get the vizualizer
    v = p.constructVisualizer();
    v.display_dt = .05;
    
    % get initial and final states and guess trajectory
    tf0 = 4;
    [x0,xf,traj_init] = getInitialConditions(p,n_arm_pos,n_arm_vel,n_block_pos,n_block_vel,n_arm_u,n_block_u,tf0);
    v.playback(traj_init.x);    
    
    % set up Dircol
    N = 21;
    d = 0.1; % distance buffer for "touching"
    blockVelOffset = n_arm_pos + n_block_pos + n_arm_vel;
    blockPosOffset = n_arm_pos;
    prog = setUpDircol(p,x0,xf,N,d,blockPosOffset,blockVelOffset);
    
    % compute the trajectory
    for attempts=1:10
        tic
        [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
        toc
        traj_init.x = xtraj;
        traj_init.u = utraj;
        v.playback(xtraj);
        if info==1
            break;
        end
    end
    
    % playback the final trajectory
    v.playback(xtraj, struct('slider', true));
    
    
    %% Running cost function
    function [g,dg] = cost(p,dt,x,u,blockPosOffset,blockVelOffset)
        % penalize for distance of hand from block
        [xerr, dXerr] = distanceHandBlock(p,x,blockPosOffset);
        Q = 10*diag(ones(1,size(xerr,1)));
        
        % penalize for finger movements
        u_size = size(u,1);
        R_diag = ones(1,u_size);
        if u_size > 7
            R_diag(8:u_size) = 50;
        end
        R = 10*diag(R_diag);
        
        % then compute total cost and gradients
        g = xerr'*Q*xerr + u'*R*u;
        dgddt = 0;
        dgdx = 2*xerr'*Q*dXerr;
        dgdu = 2*u'*R;
        dg = [dgddt,dgdx,dgdu];
    end
    
    %% Final cost function
    function [h,dh] = finalCost(p,t,x)
        % short circuit and just use time
        Qt = 1000;
        h = Qt*t;
        dh = [Qt,zeros(1,size(x,1))];
    end

    %% get the basic plant
    function [p] = getBasicPlant()
        options.floating = false;
        options.terrain = RigidBodyFlatTerrain();
        w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
        p = RigidBodyManipulator('urdf/iiwa14.urdf',options);
        warning(w);
    end

    %% add a urdf as hand to the plant
    function [p] = addHand(p,urdfPath)
        options_hand.weld_to_link = p.findLinkId('iiwa_link_7');
        options_hand.axis = [0;0;1];
        p = p.addRobotFromURDF(getFullPathFromRelativePath(urdfPath),[0;0;.05],[pi/2;0;0],options_hand);
        p = p.compile();
    end

    %% add the block to the combined plant
    function [p] = addBlock(p,urdfPath)
        options_block.floating = false;
        p = p.addRobotFromURDF(getFullPathFromRelativePath(urdfPath),[0;0;0],[0;0;0],options_block);
        p = p.compile();
    end
    

    %% set up initial conditions and guess trajectory
    function [x0,xf,traj_init] = getInitialConditions(p,n_arm_pos,n_arm_vel,n_block_pos,n_block_vel,n_arm_u,n_block_u,tf0) 
        % set initial (up) and middle (horizontal next to block) and final
        % raised up with block states with zero velocity
        x0_arm = zeros(n_arm_pos,1);
        xm_arm = zeros(n_arm_pos,1);
        xm_arm(2) = pi/2 - 0.2;
        xm_arm(6) = pi/4;
        xf_arm = x0_arm;

        x0_block = zeros(n_block_pos,1);
        x0_block(1) = 1;
        x0_block(3) = 0.2;
        xm_block = x0_block;
        xf_block = zeros(n_block_pos,1);
        xf_block(3) = 1.556;
        
        x0 = [x0_arm; x0_block; zeros(n_arm_vel+n_block_vel,1)];
        xm = [xm_arm; xm_block; zeros(n_arm_vel+n_block_vel,1)];
        xf = [xf_arm; xf_block; zeros(n_arm_vel+n_block_vel,1)];
        
        % get initial guess trajectory
        traj_init.x = computeKinematicTraj(p,x0,xm,xf,n_arm_pos);
    end

    %% function to compute kinematic trajectory
    function [xtraj] = computeKinematicTraj(p,x0,xm,xf,blockPosOffset)
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
        qm = xm(1:p.num_positions);
        qf = xf(1:p.num_positions);
        qf_block = qf(blockPosOffset+1:size(qf,1));
        hand_xyz_xm = handPos(p,xm);
        hand_xyz_xf = handPos(p,xf);
        Tf = 1;
        n = 2;
        
        % solve for x0 to xm
        tol = 0.05;
        ikCons0m{1,1} = WorldPositionConstraint(p,hand_id,pt_on_hand,hand_xyz_xm-tol,hand_xyz_xm+tol,[Tf,Tf]);
        [xtraj0m] = computeIK(p,q0,qm,ikCons0m,Tf,n);

        % solve for xm to xf
        tol = 0.05;
        ikConsmf{1,1} = WorldPositionConstraint(p,hand_id,pt_on_hand,hand_xyz_xf-tol,hand_xyz_xf+tol,[Tf,Tf]);
        ikConsmf{2,1} = WorldPositionConstraint(p,block_id,pt_on_block,qf_block-tol,qf_block+tol,[Tf,Tf]);
        [xtrajmf] = computeIK(p,qm,qf,ikConsmf,Tf,n);

        % then stich together
        xtrajmf = xtrajmf.shiftTime(Tf);
        xtraj = xtraj0m.append(xtrajmf);
    end
    
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

    %% function to return hand height
    function [z,dZ] = handHeight(p,x)
        [hand_pos, dHand_pos] = handPos(p,x);
        z = hand_pos(3);
        dZ = [dHand_pos(3,:), zeros(1,p.num_velocities)];
    end

    %% function to return middle height
    function [z,dZ] = middleHeight(p,x)
        kinsol = p.doKinematics(x(1:p.num_positions));
        hand_body = p.findLinkId('iiwa_link_4');
        pos_on_hand_body = [0;0;0];
        [hand_pos, dHand_pos] = p.forwardKin(kinsol,hand_body,pos_on_hand_body);
        z = hand_pos(3);
        dZ = [dHand_pos(3,:), zeros(1,p.num_velocities)];
    end

    %% function to retrun block height
    function [z] = ballHeight(x,blockPosOffset)
        z = x(blockPosOffset+3) - 0.2; %0.2 is radius
    end
    
    %% matrix of distance of block from hand (base) and norm
    function [xerr, dXerr, dis, dDis] = distanceHandBlock(p,x,blockPosOffset)
        [hand_pos, dHand_pos, ddHand_pos] = handPos(p,x);
        block_pos = x(blockPosOffset+1:blockPosOffset+3);
        dBlock_pos = zeros(3,p.num_positions);
        dBlock_pos(1,blockPosOffset+1) = 1;
        dBlock_pos(2,blockPosOffset+2) =1;
        dBlock_pos(3,blockPosOffset+3) = 1;
        xerr = hand_pos - block_pos;
        dXerr = [dHand_pos - dBlock_pos, zeros(3,p.num_velocities)];
        disSq = xerr'*eye(size(xerr,1))*xerr;
        dDisSq = 2*xerr'*dXerr;
        radius = 0.2;
        dis = sqrt(disSq) - radius;
        dDis = 1/2*disSq * dDisSq;
    end

    %% repetatively multiply sections of A by B
    function [C] = repmult(A,B)
        n = size(B,1);
        m = size(A,1);
        C = zeros(m,n);
        for index = 1:n
            C(:,index) = A(:,(index-1)*n+1:index*n)*B;
        end
    end

    %% function to return hand vel
    function [hand_vel,dHand_vel] = handVel(p,x)
        [hand_pos, dHand_pos, ddHand_pos] = handPos(p,x);
        qd = x(p.num_positions+1:p.num_positions+p.num_velocities);
        hand_vel = dHand_pos * qd;
        dHand_vel = [repmult(ddHand_pos,qd),dHand_pos];
    end

    %% function to to return block vel and its norm
    function [block_vel, dBlock_vel, vel, dVel] = blockVel(x,blockVelOffset)
        block_vel = x(blockVelOffset+1:blockVelOffset+3);
        dBlock_vel = zeros(1,size(x,1));
        dBlock_vel(1,blockVelOffset+1) = 1;
        dBlock_vel(2,blockVelOffset+2) = 1;
        dBlock_vel(3,blockVelOffset+3) = 1;
        vel = sqrt(block_vel'*eye(size(block_vel,1))*block_vel);
        dVel = 2*block_vel'*dBlock_vel;
    end

    %% function ensure correct block velocity
    function [res,dRes] = blockVelDist(p,x,d,blockPosOffset,blockVelOffset)
        [xerr, dXerr, dis, dDis] = distanceHandBlock(p,x,blockPosOffset);
        [block_vel, dBlock_vel, bvel, dBvel] = blockVel(x,blockVelOffset);
        res = dis*bvel;
        dRes = dis*dBvel + bvel*dDis;
        %{
        % block velocity of zero if over touching distance
        if norm(xerr) > d + 0.2 %(radius of 0.2 now)
            [block_vel, dBlock_vel] = blockVel(x,blockVelOffset);
            res = block_vel'*eye(size(block_vel,1))*block_vel;
            dRes = 1/2*block_vel'*dBlock_vel;
        % else same as vel of arm in xyz (and TBD also for rpy)
        else
            [hand_vel, dHand_vel] = handVel(p,x);
            [block_vel, dBlock_vel] = blockVel(x,blockVelOffset);
            delta_vel = hand_vel - block_vel;
            dDelta_vel = dHand_vel - dBlock_vel;
            res = delta_vel'*eye(size(delta_vel,1))*delta_vel;
            dRes = 1/2*delta_vel'*dDelta_vel;
        end
        %}
    end

    %% setup Dircol
    function [prog] = setUpDircol(p,x0,xf,N,d,blockPosOffset,blockVelOffset)
        minor = 0.001;
        major = 0.01;
        %{
        options.MinorFeasibilityTolerance = minor;
        options.MajorFeasibilityTolerance = major;
        options.MajorOptimalityTolerance = major;
        options.compl_slack = minor;
        options.lincompl_slack = major;
        options.jlcompl_slack = major;
        %}
        prog = DircolTrajectoryOptimization(p,N,[2 6]);%,options);
        % intial and final states and costs
        prog = prog.addStateConstraint(ConstantConstraint(x0),1);
        prog = prog.addStateConstraint(BoundingBoxConstraint(xf-major,xf+major),N);
        prog = prog.addRunningCost(@(dt,x,u)cost(p,dt,x,u,blockPosOffset,blockVelOffset));
        prog = prog.addFinalCost(@(t,x)finalCost(p,t,x));
        % get all states for certain constraints
        all_states = zeros(1,N);
        for index = 1:N
            all_states(index) = index;
        end
        % make sure hand doesn't penetrate the ground
        non_ground_penetration = FunctionHandleConstraint(0,inf,p.num_positions+p.num_velocities,@(x)handHeight(p,x));
        prog = prog.addStateConstraint(non_ground_penetration,all_states);
        % make sure middle of arm doesn't penetrate the ground
        non_ground_penetration_m = FunctionHandleConstraint(0,inf,p.num_positions+p.num_velocities,@(x)middleHeight(p,x));
        prog = prog.addStateConstraint(non_ground_penetration_m,all_states);
        % make sure ball doesn't go into the ground
        non_ground_penetration_b = FunctionHandleConstraint(0,inf,p.num_positions+p.num_velocities,@(x)ballHeight(x,blockPosOffset));
        prog = prog.addStateConstraint(non_ground_penetration_b,all_states);
        % make sure the block doesn't move if not touching the arm else
        % moves at same speed as the arm
        %block_constraint = FunctionHandleConstraint(-1*minor,minor,p.num_positions+p.num_velocities,@(x)blockVelDist(p,x,d,blockPosOffset,blockVelOffset));
        %prog = prog.addStateConstraint(block_constraint,all_states);
    end
end