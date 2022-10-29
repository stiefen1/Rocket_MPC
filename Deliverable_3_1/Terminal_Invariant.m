function [Ht,ht] = Terminal_Invariant(H,h,G,g,A,B,K, name)

Amod = A+B*K;
Hmod = [H;G*K];
hmod = [h;g];
W = Polyhedron(Hmod,hmod);
i = 1;

preW_all = [];

%% Iterate to find invariant set
%figure , hold on, grid on
while 1 && i<10 %%Standard = 20
    preW = Polyhedron(Hmod*Amod,hmod);
    preW_all = [preW_all; preW];
    Intersect = intersect(preW, W);
    
    if Intersect == W
        break
    else
        i = i+1;
        W = Intersect;
        Hmod = W.A;
        hmod = W.b;
    end
    
    
end
for i=1:size(Amod,2)-1
    figure
    grid on;
    hold on;
    
    for j=1:length(preW_all)
        preW_all(j).projection(i:i+1).plot('color','y','alpha', 0.2);
    end
    inv = W.projection(i:i+1).plot('color', 'm','alpha', 1);
    
    axis square
    
    if i == 1
        title(['Projection 1:2 of Invariant Set and Preset Set for ', name, ...
            '-Coordinate'], 'FontSize', 13)
    elseif i == 2
        title(['Projection 2:3 of Invariant Set and Preset Set for ', name, ...
            '-Coordinate'], 'FontSize', 13)
    elseif i == 3
        title(['Projection 3:4 of Invariant Set and Preset Set for ', name, ...
            '-Coordinate'], 'FontSize', 13)
    end
    legend([inv], 'Invariant Set')
    hold off
end
Ht = Hmod;
ht = hmod;

end