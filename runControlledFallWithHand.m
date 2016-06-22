%% run controlled fall with a hand attached to it
function runControlledFallWithHand()

    % get the plant
    options.floating = false;
    options.terrain = RigidBodyFlatTerrain();
    w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
    p = RigidBodyManipulator('urdf/iiwa14.urdf',options);
    warning(w);
    
    % add the hand
    options_hand.weld_to_link = p.findLinkId('iiwa_link_7');
    options_hand.axis = [0;0;1];
    p = p.addRobotFromURDF(getFullPathFromRelativePath('../Atlas/urdf/robotiq_simple.urdf'),[0;0;0],[pi/2;0;0],options_hand);
    p = p.compile();
    
    % get the vizualizer
    v = p.constructVisualizer();
    v.display_dt = .05;
    v.axis = [-1 1 -1 1 -1 1];
    %v.inspector();%[0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]);
    %return;

    % set initial (up) and final (horizontal) states with zero velocity
    x0 = zeros(p.num_positions + p.num_velocities,1);
    xf = zeros(p.num_positions + p.num_velocities,1);
    xf(2) = pi/2;
    
    % set up Dircol
    tf0 = 4;
    N = 21;
    prog = DircolTrajectoryOptimization(p,N,[2 6]);
    prog = prog.addStateConstraint(ConstantConstraint(x0),1);
    prog = prog.addStateConstraint(ConstantConstraint(xf),N);
    prog = prog.addRunningCost(@cost);
    prog = prog.addFinalCost(@finalCost);
    
    % get initial guess trajectory
    traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
    
    % compute the trajectory
    for attempts=1:10
        tic
        [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
        toc
        if info==1
            break;
        else
            v.playback(xtraj);
            traj_init.x = xtraj;
            traj_init.u = utraj;
        end
    end
    
    % playback the trajectory
    v.playback(xtraj, struct('slider', true));
    
    %% Running cost function
    function [g,dg] = cost(dt,x,u)
        % short circuit and just use effort
        R = diag(ones(1,size(u,1)));
        g = u'*R*u;
        dg = [zeros(1,1+size(x,1)),2*u'*R];
        return;
        
        % include x_error
        xd = verticalFixedPoint(p,v);
        xerr = x-xd;
        xerr(1,:) = mod(xerr(1,:)+pi,2*pi)-pi;

        Q = diag([1,10,ones(1,size(x,1)-2)]);
        R = 100*diag(ones(1,size(u,1)));
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
end