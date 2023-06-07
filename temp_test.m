avi_file = VideoWriter('example.avi');
open(avi_file);
f = figure('visible', 'off');
frames = 200;
x = -pi:.1:pi;
y = sin(x);
for i = 1:frames
    plot(x,y)
    frame = getframe(gcf);
    writeVideo(avi_file,frame);

    y = sin(x+(i/pi));
    fprintf(1,'%.0d of %.0d\n',i,frames)
end
close(avi_file)
implay('example.avi')