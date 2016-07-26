%% Compute the actual vertical fixed point for the KukaArm
function [xf,uf] = verticalFixedPoint(p,v)
    % guess fixed point from inspector
    %v.inspector()
    %return;
    x0 = zeros(p.num_positions + p.num_velocities,1);
    u0 = zeros(size(p.umin,1),1);
    
    % then update with fixed point program
    [xf,uf,info] = FixedPointProgram(p).findFixedPoint(x0,u0,1e-6);
    % make sure there wasn't an error
    if (info ~= 1)
        error('Guessed (x0,u0) is not close to a fixed point');
    % then remove the joint names and other extra info
    else
        for index=1:size(xf,1)
            xf(index) = xf(index);
        end
        for index=1:size(uf,1)
            uf(index) = uf(index);
        end
   end
end