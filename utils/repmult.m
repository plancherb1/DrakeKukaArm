%% repetatively multiply sections of A by B
function [C] = repmult(A,B)
    n = size(B,1);
    m = size(A,1);
    C = zeros(m,n);
    for index = 1:n
        C(:,index) = A(:,(index-1)*n+1:index*n)*B;
    end
end