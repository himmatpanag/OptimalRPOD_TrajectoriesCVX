function PlotConeConstraint(ax,alpha, height, unitVec, origin)
    if nargin < 5
        origin = [0;0;0];
    end
    axes(ax);
    heights = linspace(0,height,2);
    thetas = linspace(-pi,pi,20);
    [R,T] = meshgrid(heights,thetas);
    Z = R + origin(3); 
    X = R.*cos(T).*tan(alpha)+ origin(1);
    Y = R.*sin(T).*tan(alpha)+ origin(2);
    s = surf(X,Y,Z,'DisplayName','Cone Constraint');
    set(s,'FaceAlpha',0.3);
    set(s,'LineStyle','--');
    set(s,'EdgeAlpha',0.4);
    
    rotAngle = 180/pi * acos(dot(unitVec,[0;0;1]));   
    rotate(s,[1;0;0],-rotAngle,origin);
    axis equal
end 