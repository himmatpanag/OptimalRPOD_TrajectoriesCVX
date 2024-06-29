function [hFig1,hFig2,hFig3,hFig4,hFig5,hFig6] = CreateFigures(figVis,nameAppend)

hFig1 = figure('Name',['TrajSummary',nameAppend],'Visible',figVis); 
hFig2 = figure('Name',['Contamination',nameAppend],'Visible',figVis); grid on; hold on; 
hFig3 = figure('Name',['ContaminationRate',nameAppend],'Visible',figVis); grid on; hold on; 
hFig4 = figure('Name',['TrajPosComparison',nameAppend],'Visible',figVis); 
hFig5 = figure('Name',['TrajChange',nameAppend],'Visible',figVis); 
hFig6 = figure('Name',['Constraint',nameAppend],'Visible',figVis);

end 