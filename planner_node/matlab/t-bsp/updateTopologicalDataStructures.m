function T = updateTopologicalDataStructures(G, G_, Delta_E)


T.n_ = G_.numnodes;
T.n = G.numnodes;
T.m_ = G_.numedges;
T.m = G.numedges;
T.d_ = G_.degree;
T.d = G.degree;

% n_ x m_ prior graph incidence matrix
T.Inc_ = G_.incidence;
% prior edge weights
if T.m_ == 0
    T.edge_weights_ = [];
end
for c = 1:T.m_
    v = find(T.Inc_(:,c));
    T.edge_weights_(c) = 1/(T.d_(v(1))*T.d_(v(2)));
end


% posterior graph incidence matrix
% new edges are added at the end, i.e. edges from the prior have the same
% index in the posterior graph
T.Inc = [T.Inc_ sparse([], [], [], T.n_, size(Delta_E,1));
    sparse([], [], [], T.n-T.n_, T.m)];
newEIdxs = T.m_+ [1:size(Delta_E,1)]';
tic;idx = [sub2ind([T.n T.m], Delta_E(:,1), newEIdxs); % linear indexing
    sub2ind([T.n T.m], Delta_E(:,2), newEIdxs)];
T.Inc(idx) = 1;toc
% posterior edge weights
for c = 1:T.m
    v = find(T.Inc(:,c));
    T.edge_weights(c) = 1/(T.d(v(1))*T.d(v(2)));
end


% figure; subplot(1,2,1), spy(T.Inc_), 
% title(['Prior G: n = ' num2str(T.n_) ', m = ' num2str(T.m_)])
% subplot(1,2,2), spy(T.Inc), hold on
% plot(0.5+[T.m_, T.m_], 0.5+[T.n, 0], 'r')
% plot(0.5+[0, T.m], 0.5+[T.n_, T.n_], 'r')
% title(['Posterior G: n = ' num2str(T.n) ', m = ' num2str(T.m)])

tic; x = sum(T.Inc(1:T.n_,T.m_+1:end), 2); toc
T.V_I = find(x);

end

