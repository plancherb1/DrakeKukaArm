%% setup Dircol with initial and final states and non-ground penetration
function [prog,all_states] = setUpDircol(p,x0,xf,N,blockPosOffset,tol)
    prog = DircolTrajectoryOptimization(p,N,[2 6]);
    prog = prog.addStateConstraint(ConstantConstraint(x0),1);
    prog = prog.addStateConstraint(BoundingBoxConstraint(xf-tol,xf+tol),N);
    % get all states
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
end