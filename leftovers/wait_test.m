h = actxserver('Prok.Liberty');
h.connect;
sens=h.sensors;

data = [];
hFig = figure();
hold on;
set(hFig,'WindowStyle','docked');
t0=tic;

while ishandle(hFig), %until figure is closed
    tmp = h.position;
    if ~isempty(tmp)
        data = [data; toc(t0) tmp];
    end
end