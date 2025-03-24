%Configuração da PORTA Série em MATLAB
if ~isempty(instrfind) 
   fclose(instrfind);
   delete(instrfind);
end

close all;
clc;
s = serial('COM5','BaudRate',19200); 
set(s,'FlowControl','none');
set(s,'Parity','none');
set(s,'InputBufferSize',2);
s.Terminator="";
s.ReadAsyncMode='continuous';
set(s,'Databits',8);
set(s,'StopBit',1);
set(s,'Timeout',100);
fopen(s);
flushinput(s);

%Definição das variáveis.
i=1;
j=1;
N=1200;
a=zeros(2,1);
xa=zeros(2,1);
xv=zeros(2,1);
xx=zeros(2,1);
ya=zeros(2,1);
yv=zeros(2,1);
yy=zeros(2,1);
za=zeros(2,1);
zv=zeros(2,1);
zz=zeros(2,1);
xt=1;
yt=1;
zt=1;
ts=0.1966;

%Enlarge figure to full screen.
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);

%Ciclo de amostragem e plot.
tic
while(i<=N)
    if (i==200)
        i=1;
        a=zeros(2,1);
        x=zeros(1,1);
        xa=zeros(2,1);
        xv=zeros(2,1);
        xx=zeros(2,1);
        y=zeros(1,1);
        ya=zeros(2,1);
        yv=zeros(2,1);
        yy=zeros(2,1);
        z=zeros(1,1);
        zv=zeros(2,1);
        za=zeros(2,1);
        zz=zeros(2,1);
        xt=1;
        yt=1;
        zt=1;
    end
    for j=1:3
        idn=fscanf(s);
        int=dec2bin(idn);
        dec=bin2dec(int);
        a(1,i)=dec(1);
        a(2,i)=dec(2);
        %Distribuição dos valores de aceleração em x, y e z para
        %as respetivas listas.
        if a(1,i)== 255
            x(1,xt) = a(2,i);
            xa(2,xt)=((x(1,xt)*3.24/255)-1.58)*9.81/0.8;
            xa(1,xt)=(xt-1)*ts;
            if(xt~=1)
                %Integração dos valores de aceleração em velocidade e 
                %de seguida posição.
                xv(2,xt)=ts*trapz(xa(2,(xt-1):xt));
                xv(1,xt)=(xt-1)*ts;
                xx(2,xt)=ts*trapz(xv(2,(xt-1):xt));
                xx(1,xt)=(xt-1)*ts;
            end
            xt=xt+1;
        end
        if a(1,i) == 254
            y(1,yt) = a(2,i);
            ya(2,yt)=((y(1,yt)*3.24/255)-1.74)*9.81/0.8;
            ya(1,yt)=(yt-1)*ts;
            if(yt~=1)
                %Integração dos valores de aceleração em velocidade e 
                %de seguida posição.
                yv(2,yt)=ts*trapz(ya(2,(yt-1):yt));
                yv(1,yt)=(yt-1)*ts;
                yy(2,yt)=ts*trapz(yv(2,(yt-1):yt));
                yy(1,yt)=(yt-1)*ts;
            end
            yt=yt+1;
        end
        if a(1,i) == 253
            z(1,zt) = a(2,i);
            za(2,zt)=((z(1,zt)*3.24/255)-1.89)*9.81/0.8;
            za(1,zt)=(zt-1)*ts;
            if(zt~=1)
                %Integração dos valores de aceleração em velocidade e 
                %de seguida posição.
                zv(2,zt)=ts*trapz(za(2,(zt-1):zt));
                zv(1,zt)=(zt-1)*ts;
                zz(2,zt)=ts*trapz(xv(2,(zt-1):zt));
                zz(1,zt)=(zt-1)*ts;
            end
            zt=zt+1;
        end
    end
   %Plot das 3 coordenadas.
   plot3(xx(2,:),yy(2,:),zz(2,:),'-o')
   grid on
   xlabel('x(t) (m)','FontSize',12)
   ylabel('y(t) (m)','FontSize',12)
   zlabel('z(t) (m)','FontSize',12)
   title('Posição em função do tempo r[x(t),y(t),z(t)]:','FontSize',14)
   pause(0.00000001);
   i=i+1;
end
toc
