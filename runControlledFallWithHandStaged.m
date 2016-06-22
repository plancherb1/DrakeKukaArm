%% run controlled fall with a hand attached to it in two stages first without then with the hand to be faster
function runControlledFallWithHandStaged()
    
    % get the plant
    p = getBasicPlant();
    
    % add the block reprsenting the hand
    p = addHand(p,'../Atlas/urdf/robotiq_box.urdf');
    
    % get the vizualizer
    v = p.constructVisualizer();
    v.display_dt = .05;
    
    % set initial (up) and final (horizontal) states with zero velocity
    x0 = zeros(p.num_positions + p.num_velocities,1);
    xf = zeros(p.num_positions + p.num_velocities,1);
    xf(2) = pi/2;
    
    % set up Dircol
    N = 21;
    prog = setUpDircol(p,x0,xf,N);
    
    % get initial guess trajectory
    tf0 = 4;
    traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
    
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
    
    % now re-set up the problem with the actual hand on the plant
    p2 = getBasicPlant();
    p2 = addHand(p2,'../Atlas/urdf/robotiq_simple.urdf');
    v2 = p2.constructVisualizer();
    v2.display_dt = .05;
    x02 = zeros(p2.num_positions + p2.num_velocities,1);
    xf2 = zeros(p2.num_positions + p2.num_velocities,1);
    xf2(2) = pi/2;
    prog = setUpDircol(p2,x02,xf2,N);
    
    % now add 0s to fill out the x and u traj from before
    arm_xtraj = traj_init.x;
    arm_utraj = traj_init.u;
    hand_x = zeros((p2.num_positions + p2.num_velocities) - (p.num_positions + p.num_velocities),1);
    hand_u = zeros(size(p2.umin,1) - size(p.umin,1),1);
    hand_xtraj = PPTrajectory(foh(arm_xtraj.getBreaks,linspace2D(double(hand_x),double(hand_x),N)));
    hand_utraj = PPTrajectory(foh(arm_utraj.getBreaks,linspace2D(double(hand_u),double(hand_u),N)));
    traj_init2.x = arm_xtraj.vertcat(hand_xtraj);
    traj_init2.u = arm_utraj.vertcat(hand_utraj);
    
    % then resolve as before
    for attempts=1:10
        tic
        [xtraj2,utraj2,z,F,info] = prog.solveTraj(tf0,traj_init2);
        toc
        traj_init2.x = xtraj2;
        traj_init2.u = utraj2;
        v2.playback(xtraj2);
        if info==1
            break;
        end
    end
    
    % playback the final trajectory
    v2.playback(xtraj2, struct('slider', true));
    
    %% compute 2D linspace
    function [out] = linspace2D(x0,xf,N)
        if nargin < 3
            N = 100;
        end
        xDim = size(x0,1);
        out = zeros(xDim,N);
        for row = 1:xDim
            out(row,:) = linspace(x0(row),xf(row),N);
        end
    end
    
    %% Running cost function
    function [g,dg] = cost(dt,x,u)
        % short circuit and just use effort
        %R = diag(ones(1,size(u,1)));
        %g = u'*R*u;
        %dg = [zeros(1,1+size(x,1)),2*u'*R];
        %return;
        
        % include angle magniture do things that could possibly cause it to
        % go under ground (aka 2,4,6 joint values distance from 0)
        xerr = zeros(size(x,1),1);
        xerr(2) = x(2);
        xerr(4) = x(4);
        xerr(6) = x(6);
        Q = 10*diag(ones(1,size(x,1)));
        
        % penalize for finger movements
        u_size = size(u,1);
        R_diag = ones(1,u_size);
        if u_size > 7
            R_diag(8:u_size) = 50;
        end
        R = 100*diag(R_diag);
        
        % then compute total cost and gradients
        g = xerr'*Q*xerr + u'*R*u;
        dgddt = 0;
        dgdx = 2*xerr'*Q;
        dgdu = 2*u'*R;
        dg = [dgddt,dgdx,dgdu];
    end
    
    %% Final cost function
    function [h,dh] = finalCost(t,x)
        % short circuit and just use time
        Qt = 1000;
        h = Qt*t;
        dh = [Qt,zeros(1,size(x,1))];
        return;

        % include x_error
        xd = verticalFixedPoint(p,v);
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;

        Qf = 100*diag([1,10,ones(1,size(x,1)-2)]);
        h = Qt*t + xerr'*Q*xerr;
        dh = [Qt, 2*xerr'*Qf];
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
        p = p.addRobotFromURDF(getFullPathFromRelativePath(urdfPath),[0;0;0],[pi/2;0;0],options_hand);
        p = p.compile();
    end

    %% function to return hand height
    function [z] = handHeight(p,x)
        kinsol = p.doKinematics(x(1:p.num_positions));
        hand_body = p.findLinkId('iiwa_link_7');
        pos_on_hand_body = [0;0;0.25];
        [hand_pos, dHand_pos] = p.forwardKin(kinsol,hand_body,pos_on_hand_body);
        z = hand_pos(3);
    end

    %% setup Dircol
    function [prog] = setUpDircol(p,x0,xf,N)
        all_states = zeros(1,N);
        for index = 1:N
            all_states(index) = index;
        end
        prog = DircolTrajectoryOptimization(p,N,[2 6]);
        prog = prog.addStateConstraint(ConstantConstraint(x0),1);
        prog = prog.addStateConstraint(ConstantConstraint(xf),N);
        non_ground_penetration = FunctionHandleConstraint(0,inf,p.num_positions+p.num_velocities,@(x)handHeight(p,x));
        prog = prog.addStateConstraint(non_ground_penetration,all_states);
        prog = prog.addRunningCost(@cost);
        prog = prog.addFinalCost(@finalCost);
    end
end