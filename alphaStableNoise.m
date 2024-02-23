function xn=alphaStableNoise(alpha,n,m)

 pd1 = makedist('Stable','alpha',alpha,'beta',0,'gam',1,'delta',0.5);
%pd1 = makedist('Stable','alpha',alpha,'beta',-0.6164,'gam',2.2172,'delta',65.62);
%  pd1 = makedist('Stable','alpha',alpha,'beta',-0.435762,'gam',0.0284548,'delta',0.4061);

%n = 1000;
scale=1;
x = linspace(0,scale,n);
pdf1 = pdf(pd1,x);
cdf1 = cumsum(pdf1)/sum(pdf1);
% plot(pdf1)

% m = 1;
nbins = 50;
for i=1:m
    y = rand();
    k = find(y<=cdf1,1,'first');
    xn(i) = x(k);
    yn(i) = y;
    kn(i) = k;
end

% subplot 121
% plot(xn)
% subplot 122
% h = histogram(xn,nbins,'Normalization','probability');
% plot(h.BinEdges(1:end-1),h.Values,'r'); hold on
% plot(x,pdf1,'k'); hold off

end