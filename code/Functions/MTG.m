%%DEFINE TRAJECTORIES%%

function [Traj, Orien, NumTraj] = MTG(Position,Direction,distCrossing,TrajThr)% MultiTrajectory Generation
%%
P=Position;
D=Direction;

P = P(any(P,2),:); % Delete cero rows
D = D(any(D,2),:); % Delete cero rows
Angulo=angle(D); %Angle
sP=size(P);
Dist=[distCrossing distCrossing];

Traj=zeros(sP(2)/2,3,sP(1)); % Trayectorias, considero los retrasos en el +1
Orien=zeros(sP(2)/2,1,sP(1)); % Orientación, 
AnguloO=zeros(sP(2)/2,1,sP(1)); % Orientación, 

traj=1;
for n=1:1:(sP(2)/2)-1     % Loop por frame
    % Bucle segun direcciones, si un punto puede corresponder a dos
    % trajectorias
    for j=1:sP(1)       % Loop por punto en sigiente frame 
        for i=1:sP(1)   % Loop por ultimo punto de trajectorias     
            for k=1:sP(1)   % Segundo Loop por ultimo punto de trajectorias    
               if i~=k
                    if and(abs(Traj(n,1:2,i)-P(j,2*n+1:2*n+2)) <=Dist , abs(Traj(n,1:2,k)-P(j,2*n+1:2*n+2)) <=Dist) % Si un punto puede pertenecer a dos trajectorias diferentes

                        if abs(Angulo(j,2*n+1)-AnguloO(n,1,i))<=abs(Angulo(j,2*n+1)-AnguloO(n,1,k)) %Comparo diferencias en direccion
                            Traj(n+1,:,i) = [P(j,2*n+1:2*n+2) , 0];        % Añado punto en trajectoria
                            Orien(n+1,:,i) = D(j,2*n+1);                    % Añado punto a orientaciones
                            AnguloO(n+1,:,i) = D(j,2*n+1);                   % Añado punto a orientaciones  
                        else
                            Traj(n+1,:,k) = [P(j,2*n+1:2*n+2) , 0];        % Añado punto en trajectoria
                            Orien(n+1,:,k) = D(j,2*n+1);                    % Añado punto a orientaciones
                            AnguloO(n+1,:,k) = D(j,2*n+1);                    % Añado punto a orientaciones
                        end
                        P(j,2*n+1:2*n+2) = 0;                       % Borro punto

                    end
               end
            end
        end
    end

    % Bucle para asociar el siguiente punto
    for i=1:sP(1)       % Loop por ultimo punto de trajectorias
        if Traj(n+1,:,i)==0
            for j=1:sP(1)   % Loop por Circulo en sigiente frame  
                if abs(Traj(n,1:2,i)-P(j,2*n+1:2*n+2)) <=Dist;
                    Traj(n+1,:,i) = [P(j,2*n+1:2*n+2) , 0];        % Añado punto en trajectoria
                    Orien(n+1,:,i) = D(j,2*n+1);                    % Añado punto a orientaciones
                    P(j,2*n+1:2*n+2) = 0;                       % Borro punto
                end
            end
        end
    end
    
    % Crear trajectoria 
    for i=1:sP(1)       % Loop por ultimo punto de trajectorias
        for j=1:sP(1)   % Loop por Circulo en sigiente frame        
            if P(j,2*n+1:2*n+2)~=0
                Traj(n+1,1:2,traj)=P(j,2*n+1:2*n+2);   % Creo trajectoria con ese punto
                Orien(n+1,:,traj)=D(j,2*n+1);           % Creo traj.orientacion con ese punto
                traj=traj+1;                        % Para crear la segunda en el segundo plano
                P(j,2*n+1:2*n+2) = 0;               % Borro punto
            end
        end
    end
    
    % Bucle si son ceros
    for i=1:sP(1)                               % Loop por ultimo punto de trajectorias
        if Traj(n+1,:,i)==0;                       % Si la trajectoria no ha cambiado
            Traj(n+1,:,i) = Traj(n,:,i)+[0 0 1];      % Coloco anterior y marco retraso
        end
    end
   
    
end

%Descartar Trajectorias por ruido
    NumTraj=size(Traj,3);
    for i=NumTraj:-1:1
       if nnz(Orien(:,:,i))<=TrajThr
           Orien(:,:,i)=[];
           Traj(:,:,i)=[];

       end
    end
    Traj(1,:,:)=[];
    Orien(1,:,:)=[];
    NumTraj=size(Traj,3);
end

