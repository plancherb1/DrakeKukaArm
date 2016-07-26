%% run the lqr controller from a handful of initial conditions
function runLQR()
    % add the utils folder to the path
    utils_path = strcat(pwd, '/utils');
    addpath(utils_path);
    
    % get the plant
    p = getBasicPlant();

    % get the vizualizer
    v = p.constructVisualizer();
    v.display_dt = .05;

    % get the vertical fixed point
    [xf,uf] = verticalFixedPoint(p,v);

    % get the LQR feedback system
    Q = diag([10*ones(1,p.num_positions),ones(1,p.num_velocities)]);
    R = diag(ones(1,size(p.umin,1)));   
    c = tilqr_ignore_contacts(p,xf,uf,Q,R);

    % then create the system
    tsp = TimeSteppingRigidBodyManipulator(p,0.001); 
    sys = feedback(tsp,c);
    
    % remove the utils from the path
    rmpath(utils_path);
    
    % simulate and playback
    for i=1:5
        randX = xf+0.2*randn(size(xf,1),1);
        xtraj=simulate(sys,[0 4],randX);
        v.playback(xtraj);
    end
    
end