

for testnum=1:7

clc
% clear all
close all
% clf

format short

% Draw the Floor Plan 
floorplan = imread('B08F2d01-SuperZoomed.png');
imshow(floorplan); hold on 
M = size(floorplan,1); N = size(floorplan,2);
x=linspace(1,M,20); y=linspace(1,N,20);
xm=meshgrid(x); ym=meshgrid(y);
plot(xm,y,'c','LineWidth',1); plot(x,ym,'c','LineWidth',1)
hold on


% Put the Reference Data in structs

nRefNodes=5;

Reflist(1).HexAdd='6890';
Reflist(1).pos=[424 233]; % Pixels on the Floor Plan
Reflist(1).DecAdd=hex2dec('6890');
Temp=load('Ref6890.txt');
Reflist(1).data=Temp;
clear Temp

Reflist(2).HexAdd='0001';
Reflist(2).pos=[212 167]; % Pixels on the Floor Plan
Reflist(2).DecAdd=hex2dec('0001');
Temp=load('Ref0001_room2960.txt');
Reflist(2).data=Temp;
clear Temp


Reflist(3).HexAdd='7604';
Reflist(3).pos=[424 432]; % Pixels on the Floor Plan
Reflist(3).DecAdd=hex2dec('7604');
Temp=load('Ref7604_IFO_rm3980.txt');
Reflist(3).data=Temp;
clear Temp


Reflist(4).HexAdd='143F';
Reflist(4).pos=[530 598]; % Pixels on the Floor Plan
Reflist(4).DecAdd=hex2dec('143F');
Temp=load('Ref143f_IFO_t325.txt');
Reflist(4).data=Temp;
clear Temp


Reflist(5).HexAdd='50F5';
Reflist(5).pos=[142 67]; % Pixels on the Floor Plan
Reflist(5).DecAdd=hex2dec('50F5');
Temp=load('Ref50F5_IFO_rm3950.txt');
Reflist(5).data=Temp;
clear Temp


% Overlay the over the Floor Plan
fg1=figure(1);
title('Floor Map');
for i=1:nRefNodes
  plot(Reflist(i).pos(1),Reflist(i).pos(2),'or','LineWidth',4)
  text(Reflist(i).pos(1)-40,Reflist(i).pos(2)+30,Reflist(i).HexAdd,'Color','r','FontWeight','bold')
  hold on
end

 for k=1:nRefNodes
    % find the packet with the biggest number of visible nodes
    mx=max(Reflist(k).data(:,1));
    ind=find(Reflist(k).data(:,1) == mx);

    % sort these chosen packets with addresses in Asc order
    av_add= sort(Reflist(k).data(ind(1),2:2:end));

    j=2;
    for i=1:length(av_add)
        tmp=Reflist(k).data(ind(1),:);

        ind2=find(tmp==av_add(i));
        sorted(1,1)=Reflist(k).data(ind(1),1);

        if(~isempty(ind2))
            sorted(1,j)=Reflist(k).data(ind(1),ind2);
            j=j+1;
            sorted(1,j)=Reflist(k).data(ind(1),ind2+1);
            j=j+1;
        end
    end
%     ind(1)
%     sorted
    Reflist(k).Largest_Vect=sorted;
    clear sorted tmp   av_add  
end
% Later you can average the chosen packets ( IFF Necessary)


% Load Blond Node Data 

filenum=num2str(testnum);
header='BlindatPosB';
tailer='.txt';
filename=strcat(header,filenum,tailer);
Blind=load(filename);
 
% Tests at exactly the reference nodes
% testnum=5;
% Blind=load('Ref143f_IFO_t325.txt');
% test_loc=[389 233;
%           177 167;
%           106 67;
%           389 432;
%           495 598];
% 


test_loc=[424 67;
          283 266;
          283 167;
          142 100;
          36  200;
          354 399;
          460 565];

% for i=1:length(test_loc)
i=testnum;
    plot(test_loc(i,1),test_loc(i,2),'ob','LineWidth',5);
    text(test_loc(i,1)+10,test_loc(i,2)+10,num2str(i),'LineWidth',2,...
         'Color','b','FontWeight','bold');
% end

% Sort its packets with Asc order according to addresses of visible Refs
% not counting zeros 

for k=1:length(Blind)
    nVizNodes=Blind(k,1);
    Blindpkt=Blind(k,1:(nVizNodes*2)+1);
    BlindVizRefList=sort(Blindpkt(1,2:2:end));
    sorted=[];
    j=2;
    for i=1:nVizNodes
        ind=find(Blind(k,:)==BlindVizRefList(i));
        sorted(1,1)=Blind(k,1);
        if (~isempty(ind))
            sorted(1,j)=Blind(k,ind);
            j=j+1;
            sorted(1,j)=Blind(k,ind+1);
            j=j+1;
        end
    end
    Blind(k,1:(nVizNodes*2)+1)=sorted;
    clear sorted Blindpkt
end



 for k=1:1:length(Blind)
    % Find the closest Ref Largest vector
    % First check if the Ref has the same number of visible nodes or more
    count_save=[];
    for j=1:nRefNodes
        if (Blind(k,1)<=Reflist(j).Largest_Vect(1))
            count=0;
            for i=2:2:length(Blind(k,:))
                ind=find(Reflist(j).Largest_Vect==Blind(k,i));
                if (~isempty(ind))
                    count=count+1;
                end
            end
        else 
            count=0;
        end
           count_save=[count_save; count];        
    end
    % count_save

    % These are the indices for References with the same visible nodes or more 
    % as the blind node
    ind=find(count_save==max(count_save));

%     Reflist(ind(4)).Largest_Vect
%     Blind(k,:)
    LeastsqError=[];
    for j=1:length(ind)
        error_save=[];
        for i=2:2:2*count_save(ind(j))
            ind2=find(Reflist(ind(j)).Largest_Vect==Blind(k,i));
            error=abs(Reflist(ind(j)).Largest_Vect(ind2+1)-Blind(k,i+1));
            error_save=[error_save error];
        end
        error_save;
        LeastsqError(j,1)=sqrt((error_save(1)^2) + (error_save(2)^2) + (error_save(3)^2));
        LeastsqError(j,2)=ind(j);
    end
    
%     LeastsqError
    % Only the First Blind Sample wont work make this loop work for all samples
    % and choose the ref node then display it
    
    ind3=find(LeastsqError(:,1)==min(LeastsqError(:,1)));
    SticktoNode(k)=LeastsqError(ind3,2);
 end
 
%  SticktoNode
 Prob_Nodes_s=[];
 for i=1:length(SticktoNode)
     Prob_Nodes=Reflist(SticktoNode(i)).HexAdd;
     Prob_Nodes_s=[Prob_Nodes_s; Prob_Nodes];
 end
 
  
 for i=1:length(Prob_Nodes_s)
    Prob_Nodes_s_dec(i)=hex2dec(Prob_Nodes_s(i,1:4));
 end
 
 j=1;
  for i=1:nRefNodes
    % Find the probability of each location in the result
    ind=find(Prob_Nodes_s_dec==Reflist(i).DecAdd);
    freqq(j,1)=Reflist(i).DecAdd;
        if(~(isempty(ind)))
            freqq(j,2)=length(ind);
        else 
            freqq(j,2)=0;
        end
            j=j+1;           
  end
 
freqq(:,2)=100*(freqq(:,2)./sum(freqq(:,2)));


fg2=figure(2);
bar(freqq(:,2))
set(gca,'XTickLabel',{dec2hex(freqq(:,1))})
xlabel('Reference Node Address (Hexadecimal)')
ylabel('Probability (%)')
title('Location Probability Distribution')


 
% 
% filenum=num2str(testnum);
% pngFPheader='TestFloorPlan';
% pngProbheader='TestProb';
% pngtailer='.png';
% FPpngfilename=strcat(pngFPheader,filenum,pngtailer);
% Probpngfilename=strcat(pngProbheader,filenum,pngtailer);
% 
% saveas(fg1,FPpngfilename, 'png');
% saveas(fg2,Probpngfilename, 'png');
 pause(6)
end
 
% %%
% saveas(fg1,'TestFloorPlan5p5.png', 'png');
% saveas(fg2,'TestProb5p5.png', 'png');