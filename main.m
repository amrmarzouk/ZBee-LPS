clc
clear all
close all
% clf

% Test A 

% Fixed Grid (Gd) Distance Network

Gd=10;  % Grid Distance = 10 m


% 1. Generate a grid of m x n nodes with Grid Distance Gd

m=4;        % Number of Rows
n=m;        % Number of Columns


temp=0:Gd:(m-1)*Gd;


% Generate Reference Grid with columns as 
%
% ===============|============|============|======|==========|
% Ref_Address(i) | RefXPos(i) | RefYPos(i) | RSSI | Distance |
% ===============|============|============|======|==========|
%     Ref0       |            |            |      |          |
%     Ref1       |            |            |      |          |
%      *         |            |            |      |          |
%      *         |            |            |      |          |
%      *         |            |            |      |          |
%     Refn       |            |            |      |          |
% ===============|============|============|======|==========|

Ref_Net=zeros(length(temp)^2,6);

Ref_Net(:,1)=1:length(Ref_Net);  % Fill Reference Addresses starts with 1


% Fill RefXPos and RefYPos

Lt=length(temp);
for i=0:Lt-1
   Ref_Net(((i*Lt)+1):((i+1)*Lt),2)=temp(i+1);
   Ref_Net(((i*Lt)+1):((i+1)*Lt),3)=temp;
end




% Simulate Blind Node Trajectory

r=Gd*2.5;


theta=0:pi/200:pi/2;    % Blind Node Moves Half a Circle
N_Samples=length(theta);

Blind_True_x=r.*cos(theta);
Blind_True_y=r.*sin(theta);

Blind_Noisy_x=0.*randn(1,N_Samples)+r.*cos(theta);
Blind_Noisy_y=0.*randn(1,N_Samples)+r.*sin(theta);



% Calculate RSSI from the distances

n=5;    % propagation Coeffiecient typically between 2-5 
rp=40;  % RSSI at 1 m = 40

Input=zeros(size(Ref_Net,1),size(Ref_Net,2),N_Samples);

% Calculate Distance and Fill Input Matrix
k=1;

g=3;    % Weights for WCL W=1/(d)^g  ORIGINAL
% g=12;    % Weights for WCL W=1/(d)^g
% g=50;    % Weights for WCL W=1/(d)^g

for i=1:1:length(Blind_Noisy_x)
    for j=1:length(Ref_Net)
        
        % True Distance (Dont Delete)
        Ref_Net(j,5)=sqrt( (Blind_Noisy_x(i)-Ref_Net(j,2)).^2 + (Blind_Noisy_y(i)-Ref_Net(j,3)).^2);
        
        %RSSI
        Ref_Net(j,4)=(10.*n.*log10(Ref_Net(j,5)))+rp;  % RSSI Values from True Distance 
        
        % Distance Again But from the RSSI (Dont Delete)
        Ref_Net(j,5)= 10.^ ((Ref_Net(j,4)-rp)./(10*n));
        
        % Weights for WCL W=1/(d)^g
        Ref_Net(j,6)=1./((Ref_Net(j,5)).^g);  % RSSI Values from True Distance
    end
        Input(1:size(Ref_Net,1),1:size(Ref_Net,2),k)=Ref_Net;
        k=k+1;
end


% Generate Full Input Matrix  3D
%
%                                 k =1
% ===============|============|============|======|==========|==========|
% Ref_Address(i) | RefXPos(i) | RefYPos(i) | RSSI | Distance | W=1/(d^g)|
% ===============|============|============|======|==========|==========|
%     Ref0       |            |            |      |          |          |
%     Ref1       |            |            |      |          |          |
%      *         |            |            |      |          |          |
%      *         |            |            |      |          |          |
%      *         |            |            |      |          |          |
%     Refn       |            |            |      |          |          |
% ===============|============|============|======|==========|==========|
%
%
%                                 k =2
% ===============|============|============|======|==========|==========|
% Ref_Address(i) | RefXPos(i) | RefYPos(i) | RSSI | Distance | W=1/(d^g)|
% ===============|============|============|======|==========|==========|
%     Ref0       |            |            |      |          |          |
%     Ref1       |            |            |      |          |          |
%      *         |            |            |      |          |          |
%      *         |            |            |      |          |          |
%      *         |            |            |      |          |          |
%     Refn       |            |            |      |          |          |
% ===============|============|============|======|==========|==========|

% Weighted Centroid Algorithm   

for k=1:size(Input,3)
    % Calculated Blind Node X Position from RSSI Values
    WCL(k,1)=  (sum(Input(:,6,k).*Input(:,2,k)))./(sum(Input(:,6,k)));

    % Calculated Blind Node Y Position
    WCL(k,2)=  (sum(Input(:,6,k).*Input(:,3,k)))./(sum(Input(:,6,k)));
end


% % Plot the Grid
% figure;
% hold on
% grid on
% plot(Ref_Net(:,2),Ref_Net(:,3),'*r','LineWidth',4)
% text(Ref_Net(:,2),Ref_Net(:,3)+Gd/10,num2str(Ref_Net(:,1)));
% xlabel('Position X (m)')
% ylabel('Position Y (m)')
% plot(Blind_Noisy_x,Blind_Noisy_y,'.','LineWidth',3)         % Plot Blind Node True Trajectory
% plot(WCL(:,1),WCL(:,2),'+m','LineWidth',5)         % Plot Blind Node True Trajectory
% legend('Reference Node','Blind Node True Position')
%  
%  
%  % Error in 2D
%  figure;
%  hold on
%  grid on
%  Errorx=Blind_Noisy_x'-WCL(:,1);
%  Errory=Blind_Noisy_y'-WCL(:,2);
%  Errortotal=sqrt((Blind_Noisy_x'-WCL(:,1)).^2+ (Blind_Noisy_y'-WCL(:,2)).^2);
%  Error_max=max(Errortotal)
%  
%  %  plot(Errorx)
%  %  plot(Errory,'r')
%  plot(Errortotal,'k')
%  
 

 clearvars -except Input Blind_True_x Blind_True_y Blind_Noisy_x Blind_Noisy_y N_Samples WCL rp n g Ref_Net Gd
 
 
 %%  EKF
 
 
% Initial value
% r_hat_k = r_telde_k + gamma. 10. n . log(1+ delta_d / d_k-1) + noise
RSSI=90;
% gamma = 0.01;
 gamma = 20;%<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 
 % step size = 40 cm 
step_size=0.40; 
delta_d=step_size;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Kalman Filter Parameters and Variables  %%%%%%    
%%%%%%%%%             Initialization              %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the rssi vector to 90, this is because not all network
% references are available at all Samples. So if a reference node is not
% seen we give it 90 so its weight is less and doesnot effect the
% estimation
 
rssi_KF= ones(size(Input(:,4,1),1),1);
rssi_KF= rssi_KF.*RSSI;
d_KF= 10.^((-rp+rssi_KF)./(10*n));

K=zeros(length(rssi_KF),1);
P=100.*ones(length(rssi_KF),1);
R=0.05.*ones(length(rssi_KF),1);
Q=0.0005.*ones(length(rssi_KF),1);
%     H=1.*ones(length(rssi_KF),1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note:                                                                   %
% R<0.05 and Q>0.0005 will match RSSI with IT's RSSI.                     %
% But the RSSI is not showing the true value of RSSIdue to multi-pathing  % 
% Scattering, difraction or other factors that impact the RSSI            %
% Therefore R=0.05 and Q=0005 gives better estimation of ideal RSSI       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% updating the rssi vector for the first sample and                       %  
% calculating respected distance                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rssi_KF= Input(:,4,:);
d_KF= 10.^((-rp+rssi_KF)./(10*n));

Weights_KF=d_KF.*0;

% Assuming Number of Available Nodes are all Network Nodes <<<<<<<<<<<<<<
no_ava_ref=size(Input(:,4,1),1);        

%%
 for i=1:N_Samples-1
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % loading available references for current sampling time
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (i>1)
    %   K = P.*H./(H.*P.*H+R);
    %   P =(1-K.*H).*P;    
        K = P./(P+R);
        P =(1-K).*P;    
    end
 
    for (j=1:no_ava_ref)
 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%% Using previous calculated distance to estimate RSSI %%%%
        %%%%%% for next sampling time                              %%%%
        %%%%%% Update step                                         %%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %rssi_posteriori =rssi_apri+Kgain*(measured_rssi - estimated_rssi)
        if (i>1)
            rssi_KF(j,i)=rssi_KF(j,i)+K(j)*(Input(j,4,i)-rssi_KF(j,i));
            d_KF(j,i)=10^((rp+rssi_KF(j,i))/(10*n));
            
            % Weights for WCL Kalman Filter W=1/(d)^g
            Weights_KF(j,i)=(1./(d_KF(j,i)).^g);  % RSSI Values from True Distance
                
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Using previous calculated distance to estimate RSSI       %%%%%%
%%%%%% for next sampling time                                    %%%%%%
%%%%%% Prediction step                                           %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     rssi_KF(i+1,:) = rssi_KF(i,:)+10*n*log10((d_KF(i,:)+delta_d)./d_KF(i,:));
    if(i>1)
        rssi_KF(:,1,i+1) = rssi_KF(:,1,i+1)+gamma*10*n*log10((1+(delta_d./d_KF(:,1,i-1))));
    else
        rssi_KF(:,1,i+1) = rssi_KF(:,1,i+1);
    end
    d_KF(:,1,i+1) = 10.^((rp+rssi_KF(:,1,i+1))/(10*n)); 
    P=P+Q;
%     H=(log(10)/10*n)*10.^((rp+rssi_KF(i+1,:))/(10*n));


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 end
  
% Weighted Centroid Algorithm  with EKF 

for k=1:N_Samples
    % Calculated Blind Node X Position from RSSI Values
    WCL_KF(k,1)=  (sum(Weights_KF(:,1,k).*Input(:,2,k)))./(sum(Weights_KF(:,1,k)));

    % Calculated Blind Node Y Position
  % Calculated Blind Node X Position from RSSI Values
    WCL_KF(k,2)=  (sum(Weights_KF(:,1,k).*Input(:,3,k)))./(sum(Weights_KF(:,1,k)));
end


%% Animate Trajectory

fig = figure
set(fig,'units','normalized','outerposition',[0 0 1 1]);
grid on 
hold on

x=-5:10:35;
xg=meshgrid(x);
y=-5:10:35;
yg=meshgrid(y);

for i=1:1:N_Samples
 
    pause(0.3);
    clf
    plot(Ref_Net(:,2),Ref_Net(:,3),'*r','LineWidth',4)
    hold on

    text(Ref_Net(:,2),Ref_Net(:,3)+Gd/10,num2str(Ref_Net(:,1)));
    xlabel('Position X (m)')
    ylabel('Position Y (m)')

    % Plot Blind Node Trajectory
    plot(Blind_Noisy_x(1:i),Blind_Noisy_y(1:i),'m')         
    plot(Blind_True_x(1:i),Blind_True_y(1:i),'.k')         

    % Plot Blind Node WCL Trajectory

    plot(WCL_KF(i,1),WCL_KF(i,2),'ok','LineWidth',5) 
    plot(WCL(i,1),WCL(i,2),'+g','LineWidth',5) 

    legend('Ref. Node','Measured Pos','True Pos','WCL KF','Simple WCL')
    
    str=strcat('Simulation results in Ideal Conditions(No Environmental Interference) ,g=',num2str(g));
    title(str)
    

    grid on
    plot(xg,y,'-b','LineWidth',5);
    plot(x,yg,'-b','LineWidth',5);
    %Draw a line between each reference node and the true blind node

%         for j=1:length(Ref_Net)       
%             line([Blind_Noisy_x(i)  Ref_Net(j,2)],[Blind_Noisy_y(i) Ref_Net(j,3)]);
%         end

    for j=1:size(Input,1)
        circle([Input(j,2,i),Input(j,3,i)],Input(j,5,i),1000,':');
    end



    axis([ min(Ref_Net(:,2))-3 max(Ref_Net(:,2))+3 ...
           min(Ref_Net(:,3))-3 max(Ref_Net(:,3))+3])
    
       if i==1
           pause(10);
       end
       
end

%% Error Analysis
% figure;
% hold on
% grid on
% 
% 
% subplot(2,2,1)
% plot(WCL_KF(:,1)-WCL(:,1),'k')    
% title('Difference between WCL and WCLEKF X Position')
% 
% subplot(2,2,2)
% plot(WCL_KF(:,2)-WCL(:,2),'k')    
% title('Difference between WCL and WCLEKF Y Position')
% 
% subplot(2,2,3)
% hold on
% plot(Blind_True_x(1,:)'-WCL(:,1),'k')    
% plot(Blind_True_x(1,:)'-WCL_KF(:,1),'r')    
% legend('WCL Error','WCLEKF Error')
% title('WCL vs WCLKF Error in X Position')
% 
% subplot(2,2,4)
% hold on
% plot(Blind_True_y(1,:)'-WCL(:,2),'k')    
% plot(Blind_True_y(1,:)'-WCL_KF(:,2),'r')    
% legend('WCL Error','WCLEKF Error')
% title('WCL vs WCLKF Error in Y Position')

% plot(Blind_True_y-WCL(i,2),'.k')       

%%
