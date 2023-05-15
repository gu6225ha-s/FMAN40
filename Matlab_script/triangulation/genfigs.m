%% Close all figures and clear workspace
close all;
clear;

%% Define polygon
P = [0.4626    0.7248;
     0.1584    0.4679;
     0.4764    0.0883;
     0.6838    0.3540;
     0.3543    0.5088;
     0.5685    0.6518];

%% Example of an ear
figure(1);
drawpoly(P);
drawverts(P,4);
plot([P(3,1) P(5,1)],[P(3,2) P(5,2)],'--','Color','k','LineWidth',1);
print('poly-v4.eps','-depsc');

%% Example of non-ear
figure(2);
drawpoly(P);
drawverts(P,5);
plot([P(4,1) P(6,1)],[P(4,2) P(6,2)],'--','Color','k','LineWidth',1);
print('poly-v5.eps','-depsc');

%% Triangulate
I = tripoly(P);

%% Draw polygon
figure(3);
drawpoly(P);
drawverts(P);
print('poly-tri-init.eps','-depsc');
xl = xlim;
yl = ylim;

%% Draw each step of the triangulation process
vi = 1:size(P,1);
Pi = P;
for i = 1:size(I,1)
    figure(3+i);
    drawpoly(Pi);
    xlim(xl);
    ylim(yl);
    drawverts(Pi,find(vi==I(i,2)),vi);
    T = [P(I(i,1),:);P(I(i,2),:);P(I(i,3),:)];
    h = drawpoly(T);
    h.FaceColor = "#77AC30";
    print(sprintf('poly-tri-%d.eps',i),'-depsc');
    Pi(vi==I(i,2),:) = [];
    vi(vi==I(i,2)) = [];
end

%% Draw full triangulation
figure(8);
drawverts(P);
for i = 1:size(I,1)
    T = [P(I(i,1),:);P(I(i,2),:);P(I(i,3),:)];
    h = drawpoly(T);
end
print('poly-tri-final.eps','-depsc');

%% Helper functions
function h = drawpoly(P)
    h = plot(polyshape(P),'LineWidth',2);
    set(gca,'LooseInset',get(gca,'TightInset'));
    axis off;
end

function drawverts(P,v,n)
    if ~exist('n','var')
        n = 1:size(P,1);
    end
    hold on;
    for j = 1:size(P,1)
        i = 1+mod(j-2,size(P,1));
        k = 1+mod(j,size(P,1));
        u = P(j,:)-0.5*(P(i,:)+P(k,:));
        p = P(j,:)+0.025*u/norm(u);
        if exist('v','var') && v == j
            str = sprintf('$\\mbox{\\boldmath $v_%d$}$',n(j));
        else
            str = sprintf('$v_%d$',n(j));
        end
        text(p(1),p(2),str,'FontSize',24,...
            'HorizontalAlignment','center','Interpreter','latex');
    end
end