function PlotThrustDirection(x,uDir,delta,fMag,maxNumberArrows,c)

if nargin < 6
    c = 'r';
end 
if nargin < 5 
    maxNumberArrows = 20; 
end 

if nargin < 4 
    fMag = true; 
end 

tol = 1e-4; 
idx = find(delta>tol);

% downsample
if numel(idx) > maxNumberArrows
    selection = floor(linspace(1,numel(idx),maxNumberArrows));
    idx = idx(selection);
end 

if fMag
    uDir = uDir.*delta;
end 

quiver3(x(idx, 1), x(idx, 2), x(idx, 3), uDir(idx,1), uDir(idx, 2), uDir(idx, 3), ...
    0.7, 'color', c, 'linewidth', 1.5,'DisplayName','ControlDir')

end