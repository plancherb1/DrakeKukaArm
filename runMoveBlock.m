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
    [x0,xf,traj_init] = getInitialConditions(n_arm_pos,n_arm_vel,n_block_pos,n_block_vel,n_arm_u,n_block_u,tf0);
    
    % set up Dircol
    N = 21;
    d = 0.01; % distance buffer for "touching"
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
        Q = diag(ones(1,size(xerr,1)));
        
        % penalize for block velocuty
        bv = [zeros(blockVelOffset,1);x(blockVelOffset+1:size(x,1))];
        Qbv = 10*diag(ones(1,size(bv,1)));
        
        % penalize for finger movements
        u_size = size(u,1);
        R_diag = ones(1,u_size);
        if u_size > 7
            R_diag(8:u_size) = 50;
        end
        R = 10*diag(R_diag);
        
        % then compute total cost and gradients
        g = xerr'*Q*xerr + bv'*Qbv*bv + u'*R*u;
        dgddt = 0;
        dgdx = 2*xerr'*Q*dXerr + 2*bv'*Qbv;
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
        options_block.floating = true;
        p = p.addRobotFromURDF(getFullPathFromRelativePath(urdfPath),[0;0;0],[0;0;0],options_block);
        p = p.compile();
    end

    %% set up initial conditions and guess trajectory
    function [x0,xf,traj_init] = getInitialConditions(n_arm_pos,n_arm_vel,n_block_pos,n_block_vel,n_arm_u,n_block_u,tf0) 
        % set initial (up) and middle (horizontal next to block) and final
        % raised up with block states with zero velocity
        x0_arm = zeros(n_arm_pos,1);
        xm_arm = zeros(n_arm_pos,1);
        xm_arm(2) = pi/2;
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
        
        u0 = zeros(n_arm_u+n_block_u,1);
        um = u0;
        um(2) = 1;
        um(n_arm_u+3) = 1;
        uf = u0;
        
        % get initial guess trajectory
        traj_init.x = PPTrajectory(foh([0,tf0/2,tf0],[double(x0),double(xm),double(xf)]));
        traj_init.u = PPTrajectory(foh([0,tf0/2,tf0],[double(u0),double(um),double(uf)]));
    end
    
    %% function to return hand pos
    function [hand_pos, dHand_pos] = handPos(p,x)
        kinsol = p.doKinematics(x(1:p.num_positions));
        hand_body = p.findLinkId('iiwa_link_7');
        pos_on_hand_body = [0;0;0.25];
        [hand_pos, dHand_pos] = p.forwardKin(kinsol,hand_body,pos_on_hand_body);
    end

    %% function to return hand height
    function [z] = handHeight(p,x)
        [hand_pos, dHand_pos] = handPos(p,x);
        z = hand_pos(3);
    end

    %% function to return middle height
    function [z] = middleHeight(p,x)
        kinsol = p.doKinematics(x(1:p.num_positions));
        hand_body = p.findLinkId('iiwa_link_4');
        pos_on_hand_body = [0;0;0];
        [hand_pos, dHand_pos] = p.forwardKin(kinsol,hand_body,pos_on_hand_body);
        z = hand_pos(3);
    end

    %% function to retrun block height
    function [z] = ballHeight(x,blockPosOffset)
        z = x(blockPosOffset+3);
    end

    %% 3D distance function
    function [d] = distance3D(x1,x2)
        d = sqrt((x1(1)-x2(1))^2 + (x1(2)-x2(2))^2 + (x1(3)-x2(3))^2);
    end
    
    %% matrix of distance of block from hand (base)
    function [xerr, dXerr] = distanceHandBlock(p,x,blockPosOffset)
        [hand_pos, dHand_pos] = handPos(p,x);
        block_pos = x(blockPosOffset+1:blockPosOffset+3);
        dBlock_pos = zeros(3,p.num_positions);
        dBlock_pos(1,blockPosOffset+1) = 1;
        dBlock_pos(2,blockPosOffset+2) = 1;
        dBlock_pos(3,blockPosOffset+3) = 1;
        xerr = hand_pos - block_pos;
        dXerr = [dHand_pos - dBlock_pos, zeros(3,p.num_velocities)];
    end

    %% function to return hand vel
    function [hand_vel] = handVel(p,x)
        [hand_pos, dHand_pos] = handPos(p,x);
        hand_vel = dHand_pos * x(p.num_positions+1:p.num_positions+p.num_velocities);
    end

    %% function ensure correct block velocity
    function [vel] = blockVelDist(p,x,d,blockPosOffset,blockVelOffset)
        [hand_pos, dHand_pos] = distanceHandBlock(p,x,blockPosOffset);
        distance = distance3D(hand_pos,[0;0;0]) - 0.2; %(radius of 0.2 now)
        % block velocity of zero if over touching distance
        if distance > d
            vel = double(sum(x(blockVelOffset+1:size(x,1))));
        % else same as vel of arm in xyz (and TBD also for rpy)
        else
            hand_vel = handVel(p,x);
            block_vel = x(blockVelOffset+1:blockVelOffset+3);
            vel = double(sum(hand_vel-block_vel));
            return
            % need to equate velocities and accelerations and fix the below
            q = x(1:p.num_positions);
            qd = x(p.num_positions+1:p.num_positions+p.num_velocities);
            kinsol = p.doKinematics(q, false, false, qd);
            kinsol.twists = p.twists(kinsol.T, q, qd);
            base = p.findLinkId('iiwa_link_0');
            world = base;
            body = p.findLinkId('iiwa_link_7');
            body_twist_in_world = relativeTwist(kinsol, world, body, world);
            gJLink = p.geometricJacobian(kinsol, base, body, base) *  qd(base:body-1);
            % make sure the vel and orientation vel of arm is same as ball
            delta = double(sum(x(blockVelOffset+1:size(x,1)) - [gJLink(1:3);body_twist_in_world(4:6)]));
            if delta > 0.1 % give some error bound
                vel = 1;
            else
                vel = 0;
            end
        end
    end

    %% setup Dircol
    function [prog] = setUpDircol(p,x0,xf,N,d,blockPosOffset,blockVelOffset)
        prog = DircolTrajectoryOptimization(p,N,[2 6]);
        % intial and final states and costs
        prog = prog.addStateConstraint(ConstantConstraint(x0),1);
        prog = prog.addStateConstraint(ConstantConstraint(xf),N);
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
        % moves at same speed as the arm (TBD)
        block_constraint = FunctionHandleConstraint(0,0.1,p.num_positions+p.num_velocities,@(x)blockVelDist(p,x,d,blockPosOffset,blockVelOffset));
        prog = prog.addStateConstraint(block_constraint,all_states);
    end
end