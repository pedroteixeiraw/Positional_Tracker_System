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
xt=1;
yt=1;
zt=1;
ts=0.011;
a=zeros(2,1);
x=zeros(1,1);
xa=zeros(2,1);
y=zeros(1,1);
ya=zeros(2,1);
z=zeros(1,1);
za=zeros(2,1);

%Enlarge figure to full screen.
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);

%Ciclo de amostragem e plot.
tic
while(i<=N/12)
    %Cilco de 'REFRESH' do plot após N/12 amostragens.
    if (i==N/12)
        i=1;
        a=zeros(2,1);
        x=zeros(1,1);
        xv=zeros(2,1);
        y=zeros(1,1);
        yv=zeros(2,1);
        z=zeros(1,1);
        zv=zeros(2,1); 
        xt=1;
        yt=1;
        zt=1;
    end
    %Distribuição dos valores de aceleração em x, y e z para
    %as respetivas listas.
    while(j<=12)
       idn=fscanf(s);
       int=dec2bin(idn);
       dec=bin2dec(int);
       a(1,i)=dec(1);
       a(2,i)=dec(2);
       if a(1,i)== 255
        x(1,xt) = a(2,i);
        %Conversão de valor binário em aceleraçãode x.
        xa(2,xt)= ((x(1,xt)*3.24/255)-1.65)*9.81/0.8;
        xa(1,xt)= (xt-1)*ts;
        xt=xt+1;
       end
       if a(1,i) == 254
        y(1,yt) = a(2,i);
        %Conversão de valor binário em aceleraçãode y.
        ya(2,yt)= ((y(1,yt)*3.24/255)-1.74)*9.81/0.8;
        ya(1,yt)=(yt-1)*ts;
        yt=yt+1;
        
       end
       if a(1,i) == 253
        z(1,zt) = a(2,i);
        %Conversão de valor binário em aceleraçãode z.
        za(2,zt)= ((z(1,zt)*3.24/255)-1.89)*9.81/0.8;
        za(1,zt)= (zt-1)*ts;
        zt=zt+1;
       end
       j=j+1;
    end
   %Plot das acelerações dos 3 eixos.
   j=1;
   subplot(311)
   plot(xa(1,4:4:i*4),xa(2,4:4:i*4),'r')
   xlabel('Tempo (s)')
   ylabel('a_x (m/s^2)','FontSize',12)
   title('Gráfico da Aceleração em X:','FontSize',14)
   axis([0 ((N/3-1)*ts) -24 24])
   subplot(312)
   plot(ya(1,4:4:i*4),ya(2,4:4:i*4),'b')
   xlabel('Tempo (s)')
   ylabel('a_x (ms^2)','FontSize',12)
   title('Gráfico da Aceleração em Y:','FontSize',14)
   axis([0 ((N/3-1)*ts) -24 24])
   subplot(313)
   plot(za(1,4:4:i*4),za(2,4:4:i*4),'k')
   xlabel('Tempo (s)')
   ylabel('a_x (m/s^2)','FontSize',12)
   title('Gráfico da Aceleração em Z:','FontSize',14)
   axis([0 ((N/3-1)*ts) -24 24])
   pause(0.000001);
   i=i+1;
end
toc
