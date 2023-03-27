function pendulum_anim(state,l,W,H)

x = state(1);
phi = state(3);

y = H/2;
pendx = x-l*sin(phi+pi);
pendy = y-l*cos(phi+pi);
mr = 0.1;

plot([-10,10],[0,0],'k','LineWidth',2)
hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5)
plot([x pendx],[y pendy],'k','LineWidth',4)
grid on

axis([-2 2 -1 1.5])
axis equal
set(gcf,'Position',[100 100 1000 400])
drawnow
hold off

end