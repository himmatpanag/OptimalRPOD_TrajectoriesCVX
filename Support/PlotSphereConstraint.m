function s = PlotSphereConstraint(ax,radius,origin)
    if nargin < 3
        origin = [0;0;0];
    end
    axes(ax);
    phis = linspace(-pi/2,pi/2,15);
    thetas = linspace(-pi,pi,15);
    [t,p] = meshgrid(thetas,phis);
    Z = radius.*sin(p) + origin(3); 
    X = radius.*cos(p).*cos(t)+ origin(1);
    Y = radius.*cos(p).*sin(t)+ origin(2);
    s = surf(X,Y,Z,'DisplayName',['Rad=',num2str(radius)]);
    set(s,'FaceAlpha',0.3);
    set(s,'LineStyle','--');
    set(s,'EdgeAlpha',0.3);

end 