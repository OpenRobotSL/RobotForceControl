%梁政的MDH，GLUON-6L3模型
L1=Link('d',105.03/1000,'a',0,     'alpha',0,    'offset',pi/2,'modified');
L2=Link('d',80.09/1000, 'a',0,     'alpha',pi/2, 'offset',pi/2,'modified');
L3=Link('d',0,     'a',174.42/1000,'alpha',0,    'offset',0,   'modified');
L4=Link('d',4.44/1000,  'a',174.42/1000,'alpha',pi,   'offset',pi/2,'modified');
L5=Link('d',-80.09/1000,'a',0,     'alpha',-pi/2,'offset',0,   'modified');
L6=Link('d',-44.36/1000,'a',0,     'alpha',pi/2, 'offset',0,   'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6]);
theta=[30 50 30 30 30 30]*pi/180;
theta_offset=[30 120 60 20 50 30]'*pi/180;
theta_forplot=theta_offset;
%拖动模型 Fe=Kxe.  （F=K*V）
K=10;
a=zeros(20,2);  b=8*ones(20,1); c=zeros(20,3); 
d=zeros(30,6);
a1=zeros(30,2);  b1=-10*ones(30,1); c1=zeros(30,3); 
d1=zeros(20,6);
Fe=[a,b,c;d;
   a1,b1,c1;d1 ];     %产生的100行6列的矩阵,且只在前半段时间z轴方向有向下的力，后50行无力，后期得改为六维力的数据
Fxe=Fe(:,1);Fye=Fe(:,2);Fze=Fe(:,3);Txe=Fe(:,4);Tye=Fe(:,5);Tze=Fe(:,6);
%设定初始位移，速度,加速度增量均为0
dt=0.01 ;  %时间间隔0.01
detq=[0 0 0 0 0 0]';

for i=1:99      
    dxe(i)=Fxe(i)/K;
    dye(i)=Fye(i)/K;
    dze(i)=Fze(i)/K;
    dTxe(i)=Txe(i)/K;
    dTye(i)=Tye(i)/K;
    dTze(i)=Tze(i)/K;
   
    j=robot.jacobn(theta_forplot);
    qv=inv(j)*[dxe(i),dye(i),dze(i),dTxe(i),dTye(i),dTze(i)]'; 
    detq=detq+qv*dt;
    

    theta_chuandi=theta_offset;         
    theta_chuandi=theta_chuandi+detq;
    theta_forplot=theta_chuandi';
    qq(i,:)=theta_forplot;
   
end 
 robot.plot( qq)