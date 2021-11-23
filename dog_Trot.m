%% 四足trot步态移动的角度曲线，配合adams使用
clear;
clc;

%% 不同运动顺序测试
% 读取参数测试顺序准备->下蹲->起立->踏步->前进->后退->左移->右移
%A=readmatrix('prepare.txt');  %准备
%B=readmatrix('updown.txt');  %下蹲起立
%D=readmatrix('tabu.txt');     %踏步
E=readmatrix('forward.txt');  %前进
F=readmatrix('back.txt');     %后退
G=readmatrix('left.txt');     %左移
H=readmatrix('right.txt');    %右移
angle=[E;G;F;H];
% writematrix(angle(:,1:13),'T_angle.txt','Delimiter','tab');
% writematrix(angle(:,14:end),'T_trot.txt','Delimiter','tab');
% t=angle(:,13);
% t=0.001:0.001:size(angle(:,1))/1000; %1ms执行一次


%%
% 输出足尖相对地面的坐标
quiver3(0,0,0, 100, 0, 0, 'k','linewidth',10);
quiver3(0,0,0, 0, 100, 0, 'k','linewidth',10);
quiver3(0,0,0, 0, 0, 100, 'k','linewidth',10);
plot3(0, 0, 0 ,'o','MarkerFaceColor','k','MarkerSize',6);
hold on;
grid on;

for i=1:1:size(angle(:,1))

     if rem(i,20)==0

     x  = angle(i,26);
     y  = angle(i,27);
     z  =angle(i,28);
    x1 = angle(i,14);
    y1 = angle(i,15);
    z1 = angle(i,16);
    x2 = angle(i,17);
    y2 = angle(i,18);
    z2 = angle(i,19);
    x3 = angle(i,20);
    y3 = angle(i,21);
    z3 = angle(i,22);
    x4 = angle(i,23);
    y4 = angle(i,24);
    z4 = angle(i,25);

     axis ([-2000 500   -500  2000  0 600]);


    quiver3(z,x,y, 0, 0, 20, 'r','linewidth',2);
    quiver3(z,x,y, 20, 0, 0, 'b','linewidth',2);
    quiver3(z,x,y, 0, 20, 0, 'g','linewidth',2);
    plot3(z, x, y ,'o','MarkerFaceColor','g','MarkerSize',6);

    plot3(z1,x1,y1,'o','MarkerFaceColor','r','MarkerSize',4);
    plot3(z2,x2,y2,'*','MarkerFaceColor','b','MarkerSize',6);
    plot3(z3,x3,y3,'o','MarkerFaceColor','r','MarkerSize',4);
    plot3(z4,x4,y4,'*','MarkerFaceColor','b','MarkerSize',6);
    hold on;
    grid on;
    drawnow;
    end
end



%% 输出关节角度曲线
% leg1_theta1 = angle(:,1);
% leg1_theta2 = angle(:,2);
% leg1_theta3 = angle(:,3);
% subplot(331);
% plot(t,leg1_theta1,'r',t,leg1_theta2,'b',t,leg1_theta3,'g');
% title('');
% title('Leg1 Joint Angle');
% xlabel('t/s');
% ylabel('angle/rad');
% legend('J1','J2','J3');
% 
% leg2_theta1 = angle(:,4);
% leg2_theta2 = angle(:,5);
% leg2_theta3 = angle(:,6);
% subplot(332);
% plot(t,leg2_theta1,'r',t,leg2_theta2,'b',t,leg2_theta3,'g');
% title('Leg2 Joint Angle');
% xlabel('t/s');
% ylabel('angle/rad');
% legend('J1','J2','J3');
% 
% leg3_theta1 = angle(:,7);
% leg3_theta2 = angle(:,8);
% leg3_theta3 = angle(:,9);
% subplot(334);
% plot(t,leg3_theta1,'r',t,leg3_theta2,'b',t,leg3_theta3,'g');
% title('Leg3 Joint Angle');
% xlabel('t/s');
% ylabel('angle/rad');
% legend('J1','J2','J3');
% 
% leg4_theta1 = angle(:,10);
% leg4_theta2 = angle(:,11);
% leg4_theta3 = angle(:,12);
% subplot(335);
% plot(t,leg4_theta1,'r',t,leg4_theta2,'b',t,leg4_theta3,'g');
% title('Leg4 Joint Angle');
% xlabel('t/s');
% ylabel('angle/rad');
% legend('J1','J2','J3');
% 
% %% 输出足尖末端曲线
% x1 = angle(:,14);
% y1 = angle(:,15); 
% z1 = angle(:,16);
% x2 = angle(:,17);
% y2 = angle(:,18);
% z2 = angle(:,19);
% x3 = angle(:,20);
% y3 = angle(:,21);
% z3 = angle(:,22);
% x4 = angle(:,23);
% y4 = angle(:,24);
% z4 = angle(:,25);
% x = angle(:,26);
% y = angle(:,27);
% z = angle(:,28);
% % subplot(233);
% % plot(t,x1,'r');
% %leg1
% subplot(333);
% plot(t,x1,'r',t,x2,'b',t,x,'k');
% title('Leg1 & Leg2 in x Direction');
% xlabel('t/s');
% ylabel('displacement/mm');
% legend('Leg1','Leg2','Body');
% 
% subplot(339);
% plot(t,y1,'r',t,y2,'b',t,y,'k');
%  title('Leg1 & Leg2 in z Direction');
%  legend('Leg1','Leg2','Body');
% xlabel('t/s');
% ylabel('displacement/mm');
%  
% subplot(336);
% plot(t,z1,'r',t,z3,'b',t,z,'k');
% title('Leg1 & Leg22 in y Direction');
% legend('Leg1','Leg2','Body');
% xlabel('t/s');
% ylabel('displacement/mm');
% 
