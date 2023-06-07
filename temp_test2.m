% animatedline(Ax(i),xData,yData);

xData = 0:0.001:60;
yData = sin(2*pi*xData);
xLimit = [0,5];
figure;
nAxes = 4; % Number of Axes
for i = 1:nAxes
    Ax(i) = subplot(nAxes,1,i);
    line('parent',Ax(i),'xdata',xData,'ydata',yData);
end

% Manual layout for n vertically stacked subplots
Ax(i) = axes;
Ax(i).OuterPosition = [0.0; (i-1)*0.25; 1.0; 0.25];
% Turn off toolbar and default interactivity
Ax(i).Toolbar = [];
disableDefaultInteractivity(Ax(i));
% Draw in inner position
drawnow
Ax(i).PositionConstraint = 'innerposition';



step = 0.1;
while xLimit(2)<xData(end)
    xLimit=xLimit+step;
     for i=1:nAxes
        xlim(Ax,xLimit)
     end
    pause(0.00001);
end


