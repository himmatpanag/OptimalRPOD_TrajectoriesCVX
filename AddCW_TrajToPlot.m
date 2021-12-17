function hFig = AddCW_TrajToPlot(x,ax,dispName)
    axes(ax);
    hFig = gcf;
    grid on; hold on;
    plot3(x(3,:),x(2,:),x(1,:),'DisplayName',dispName)
    axis equal; legend('show','Location','southeast')
    zlabel('R-bar (km)'), ylabel('V-bar (km)'),xlabel('Cross Track)')
    view([-135,20])
end