function G = mrpln41_SparsifyGraph(G0)

G = G0;
%figure(321)
%subplot(1,2,1), plot(G0)

for i=1:G.numnodes
    i
    n = G.neighbors(i);
    n = n(find(abs(n-i)~=1)); % stay connected, remove consecutive nodes from consideration
    
    n_before = n(find(n-i<0))';
    n_after = n(find(n-i>0))';
    n = [n_before flip(n_after)];
    
    d = diff(n);   
    % remove edges (i,j)
    idx_j = find(abs(d) == 1)+1;
    
    G = rmedge(G,i, n(idx_j));
    
end

subplot(1,2,2), plot(G)
    