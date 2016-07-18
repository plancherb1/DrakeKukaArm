%% controlled fall to block followed by raising it up computed in two parts
function runMoveBlockStaged()

    % add the utils folder to the path
    utils_path = strcat(pwd, '/utils');
    addpath(utils_path);

    % get the plant
    p = getBasicPlant();
        
    % add the block reprsenting the hand
    p = addHand(p,'../../Atlas/urdf/robotiq_box.urdf');
    n_arm_pos = p.num_positions;
    n_arm_vel = p.num_velocities;
    n_arm_u = size(p.umin,1);
    
    % add the actuated block
    p = addBlock(p,'../urdf/XYZActuatedBlock.urdf');
    n_block_pos = p.num_positions - n_arm_pos;
    n_block_vel = p.num_velocities - n_arm_vel;
    n_block_u = size(p.umin,1) - n_arm_u;
    
    % get the vizualizer
    v = p.constructVisualizer();
    v.display_dt = .05;
    
    % get initial, middle and final states
    tf0 = 4;
    [x0,xm,xf] = getInitialConditions(p,n_arm_pos,n_arm_vel,n_block_pos,n_block_vel);
    
    % get initial guess trajectories
    [traj_init0M.x] = computeKinematicTraj(p,x0,xm,n_arm_pos);
    [traj_initMF.x] = computeKinematicTraj(p,xm,xf,n_arm_pos);
    %v.playback(traj_init0M.x);
    %v.playback(traj_initMF.x);
    
    % set up the first Dircol
    N = 21;
    tol = 0;
    [prog, all_states] = setUpDircol(p,x0,xm,N,n_arm_pos,tol);
    % add costs and custom constraints
    prog = addCostsAndConstraints0M(p,prog,n_arm_pos,n_arm_pos+n_block_pos+n_arm_vel,n_arm_u,all_states);
    
    % compute the trajectory
    for attempts=1:10
        tic
        [xtraj0M,utraj0M,z,F,info] = prog.solveTraj(tf0,traj_init0M);
        toc
        traj_init0M.x = xtraj0M;
        traj_init0M.u = utraj0M;
        v.playback(xtraj0M);
        if info==1
            break;
        end
    end
    
    % set up the seecond Dircol
    N = 21;
    tol = 0;
    [prog, all_states] = setUpDircol(p,xm,xf,N,n_arm_pos,tol);
    % add costs and custom constraints
    prog = addCostsAndConstraintsMF(p,prog,n_arm_pos,n_arm_pos+n_block_pos+n_arm_vel,n_arm_u,all_states);
    
    % compute the trajectory
    for attempts=1:10
        tic
        [xtrajMF,utrajMF,z,F,info] = prog.solveTraj(tf0,traj_initMF);
        toc
        traj_initMF.x = xtrajMF;
        traj_initMF.u = utrajMF;
        v.playback(xtrajMF);
        if info==1
            break;
        end
    end
    
    % concat the final trajectories
    xtrajMF = xtrajMF.shiftTime(xtraj0M.tspan(2));
    xtraj = xtraj0M.append(xtrajMF);
    
    % remove the utils from the path
    rmpath(utils_path);
    
    % playback the final trajectory
    v.playback(xtraj, struct('slider', true));
    
    %% set up initial conditions
    function [x0,xm,xf] = getInitialConditions(p,n_arm_pos,n_arm_vel,n_block_pos,n_block_vel) 
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
    end
    
    %% Running cost from 0 to M
    function [g,dg] = cost0M(p,dt,x,u,n_arm_u)
        % include angle magniture do things that could possibly cause it to
        % go under ground (aka 2,4,6 joint values distance from 0)
        xpen = zeros(size(x,1),1);
        xpen(2) = x(2);
        xpen(4) = x(4);
        xpen(6) = x(6);
        Q = diag(ones(1,size(x,1)));
        
        % penalize for effort and hugle for the values for the block to
        % encourage it to not move which is the correct answer
        r_diag = ones(1,size(u,1));
        r_diag(n_arm_u+1:size(u,1)) = 10000;
        R = diag(r_diag);
        
        % then compute total cost and gradients
        g = xpen'*Q*xpen + u'*R*u;
        
        dgddt = 0;
        dgdx = 2*xpen'*Q;
        dgdu = 2*u'*R;
        dg = [dgddt,dgdx,dgdu];
    end

    %% Running cost function for M to F
    function [g,dg] = costMF(p,dt,x,u,blockPosOffset)
        % penalize for distance of hand from block
        [dis, dDis] = distanceHandBlock(p,x,blockPosOffset,0.2,0);
        Q = 10*diag(ones(1,size(dis,1)));
        
        % penalize for effort
        R = 10*diag(ones(1,size(u,1)));
        
        % then compute total cost and gradients
        g = dis'*Q*dis + u'*R*u;
        
        dgddt = 0;
        dgdx = 2*dis'*Q*dDis;
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

    %% function to add custom constraints and cost functions
    function [prog] = addCostsAndConstraints0M(p,prog,blockPosOffset,blockVelOffset,n_arm_u,all_states)
        prog = prog.addRunningCost(@(dt,x,u)cost0M(p,dt,x,u,n_arm_u));
        prog = prog.addFinalCost(@(t,x)finalCost(p,t,x));
        % make sure the block doesn't move while not touching arm
        block_constraint = FunctionHandleConstraint(0,0,p.num_positions+p.num_velocities,@(x)blockVel(x,blockVelOffset,1));
        prog = prog.addStateConstraint(block_constraint,all_states);
    end

    %% function to add custom constraints and cost functions
    function [prog] = addCostsAndConstraintsMF(p,prog,blockPosOffset,blockVelOffset,n_arm_u,all_states)
        prog = prog.addRunningCost(@(dt,x,u)cost(p,dt,x,u,blockPosOffset));
        prog = prog.addFinalCost(@(t,x)finalCost(p,t,x));
        % make sure the block moves at same speed as the arm
        block_constraint = FunctionHandleConstraint(0,0,p.num_positions+p.num_velocities,@(x)handBlockVelError(p,x,blockVelOffset,1));
        prog = prog.addStateConstraint(block_constraint,all_states);
    end
end