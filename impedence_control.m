%梁政的MDH，GLUON-6L3模型
clear all
L1=Link('d',105.03/1000,'a',0,     'alpha',0,    'offset',pi/2,'modified');
L2=Link('d',80.09/1000, 'a',0,     'alpha',pi/2, 'offset',pi/2,'modified');
L3=Link('d',0,     'a',174.42/1000,'alpha',0,    'offset',0,   'modified');
L4=Link('d',4.44/1000,  'a',174.42/1000,'alpha',pi,   'offset',pi/2,'modified');
L5=Link('d',-80.09/1000,'a',0,     'alpha',-pi/2,'offset',0,   'modified');
L6=Link('d',-44.36/1000,'a',0,     'alpha',pi/2, 'offset',0,   'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6]);
theta_offset=[0 0 90 0 -90 0]'*pi/180;
theta_forplot=theta_offset';
robot.plot(theta_offset')
hold on
%阻抗模型 Fe=Mxe..+Bxe.+Kxe  (其中M,B,K分别为质量，阻尼，刚度)
M=1;  B=10;  K=30;
a=zeros(50,2);  b=5*ones(50,1); c=zeros(50,3); d=zeros(150,6);
Fe=[a,b,c;d];     %产生的100行6列的矩阵,且只在前半段时间z轴方向有向下的力，后50行无力，后期得改为六维力的数据
Fxe=Fe(:,1);Fye=Fe(:,2);Fze=Fe(:,3);Txe=Fe(:,4);Tye=Fe(:,5);Tze=Fe(:,6);
%设定初始位移，速度,加速度增量均为0
dt=0.01 ;  %时间间隔0.01
detq=[0 0 0 0 0 0]';
xe(1)=0;  dxe(1)=0;  ddxe(1)=0;                        %x轴方向力引起的位移，速度，加速度增量初始值均为0
ye(1)=0;  dye(1)=0;  ddye(1)=0;   
ze(1)=0;  dze(1)=0;  ddze(1)=0; 
Txe(1)=0;  dTxe(1)=0;  ddTxe(1)=0;                     %x轴方向力矩引起的位移，速度，加速度增量初始值均为0
Tye(1)=0;  dTye(1)=0;  ddTye(1)=0;                     %y轴方向力矩同理
Tze(1)=0;  dTze(1)=0;  ddTze(1)=0;                     %z轴方向力矩同理

for i=1:199    %X轴力的增量 
    ddxe(i+1)=[Fxe(i+1)-B*dxe(i)-K*xe(i)]/M;
    dxe(i+1)=dt*[ddxe(i+1)+ddxe(i)]/2+dxe(i) ;              %v1=dt*(a0+a1)/2+v0  加速度一次积分得速度
    xe(i+1)=dt*[dxe(i+1)+dxe(i)]/2+xe(i)  ;                 %同理速度一次积分得位移
    
    %步骤同上，即y轴方向力引起关节角变化 
    ddye(i+1)=[Fye(i+1)-B*dye(i)-K*ye(i)]/M;
    dye(i+1)=dt*[ddye(i+1)+ddye(i)]/2+dye(i)   ;           
    ye(i+1)=dt*[dye(i+1)+dye(i)]/2+ye(i)    ;               
                         
    ddze(i+1)=[Fze(i+1)-B*dze(i)-K*ze(i)]/M;
    dze(i+1)=dt*ddze(i+1)+dze(i)   ;            
    ze(i+1)=dt*dze(i+1)++ze(i)  ;
        
    ddTxe(i+1)=[Txe(i+1)-B*dTxe(i)-K*Txe(i)]/M;
    dTxe(i+1)=dt*[ddTxe(i+1)+ddTxe(i)]/2+dTxe(i)   ;            
    Txe(i+1)=dt*[dTxe(i+1)+dTxe(i)]/2+Txe(i) ;
    
    ddTye(i+1)=[Tye(i+1)-B*dTye(i)-K*Tye(i)]/M;
    dTye(i+1)=dt*[ddTye(i+1)+ddTye(i)]/2+dTye(i) ;              
    Tye(i+1)=dt*[dTye(i+1)+dTye(i)]/2+Tye(i)   ; 
    
    ddTze(i+1)=[Tze(i+1)-B*dTze(i)-K*Tze(i)]/M ;
    dTze(i+1)=dt*[ddTze(i+1)+ddTze(i)]/2+dTze(i) ;              
    Tze(i+1)=dt*[dTze(i+1)+dTze(i)]/2+Tze(i) ;  
    
    j=robot.jacobn(theta_forplot);
    qv=pinv(j)*[dxe(i+1),dye(i+1),dze(i+1),dTxe(i+1),dTye(i+1),dTze(i+1)]'; %原理：detX=J(q)*detq  逆解得关节角速度的位移增量
    
    theta_offset=theta_offset+qv*dt;            %这是阻抗时的关节角
    %  theta_offset=theta_offset+detq ;       %这是拖动时关节的角度，叠加
    theta_forplot=theta_offset';
    qq(i,:)=theta_forplot;
    %robot.plot(theta_forplot)
end 
robot.plot(qq)
robot.teach