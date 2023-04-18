figure
H = drawpolygon;
P = H.Position;
I = tripoly(P);
hold on;
for i = 1:size(I,1)
    for j = 1:3
        plot([P(I(i,j),1) P(I(i,1+mod(j,3)),1)],...
            [P(I(i,j),2) P(I(i,1+mod(j,3)),2)],...
            '--','Color','k','LineWidth',1);
    end
end