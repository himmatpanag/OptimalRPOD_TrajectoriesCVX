function s = PlotCircleConstraint(ax,radius,origin)
    if nargin < 3
        origin = [0;0;0];
    end
    axes(ax);
    phis = linspace(-pi,pi,100);
    X = radius.*cos(phis) + origin(1);
    Y = radius.*sin(phis) + origin(2);
    s = patch(X,Y,'b','DisplayName',['Rad=',num2str(radius)]);
    set(s,'FaceAlpha',0.2);

end 