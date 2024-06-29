function PlotTrajXY(t,x,hFig,strName)
set(0,'CurrentFigure',hFig);
subplot 211
grid on; hold on; 
plot(t,x(1,:),'DisplayName',strName);
legend('show','Location','best');

subplot 212
grid on; hold on; 
plot(t,x(2,:),'DisplayName',strName);
legend('show','Location','best');
end 