% Look at saved measurements
% Data should be in allSubIms, allCogX, allCogY

%save('data/sampledata_static_withbgSubt1.mat','allCogX','allCogY','allSubIms')
%save('data/sampledata_scan1.mat','allCogX','allCogY','allSubIms','angles')

%load('C:\Users\AI\Desktop\AIcode_pre21072015\data\sampledata_scan1-ind3_withbgSubt')


load('sampledata_scan_20151129T194022_24px_ind3')
%load('data/sampledata')
%load('data/sampledata_scan05_16px_ind3')



%allCogX=allCogXs;
%allCogY=allCogYs;

itsAScan = true;
fitReg=[300,700]; %As Indices of angles
fitReg=[100,500]; %As Indices of angles
%fitReg=[1,size(allCogX,2)];

plotMin=-1;
plotMax=1.5;
%plotMin=-10;
%plotMax=10;

subtMiddle = false; % Subtract the middle value, for nice plotting

doRot = true;
setTheta = 60 /180*pi; %Seems to be correct to <+-1 deg for Horizontal config
setTheta = (-59) /180*pi;
setTheta = (30) /180*pi;

setTheta = (32) /180*pi;

Rmat = [ cos(setTheta), -sin(setTheta) ; sin(setTheta), cos(setTheta) ];



figure(2)
clf
nSubs = size(allCogX,1);
nIts = size(allCogX,2);

if doRot
    for ii = 1:nIts
        for kk = 1:nSubs
            cur = [allCogX(kk,ii); allCogY(kk,ii)];
            new = Rmat*cur;
            allCogX(kk,ii)=new(1);
            allCogY(kk,ii)=new(2);
        end
    end
end

if subtMiddle
    for kk = 1:nSubs
        allCogX(kk,:) = allCogX(kk,:)/allCogX(kk,floor(nIts/2));
        allCogY(kk,:) = allCogY(kk,:)/allCogY(kk,floor(nIts/2));
    end
end


allXCoeffs = zeros(nSubs,2);
allYCoeffs = zeros(nSubs,2);
for k = 1:8
    subplot(4,2,k)

    hold on
    if itsAScan
        %subplot(4,4,k*2-1)
        plot(angles,allCogX(k,:),'r');
        %axis([angles(1),angles(nIts),plotMin,plotMax])
        axis auto;
        title(num2str(k))
        
        %subplot(4,4,k*2)
        plot(angles,allCogY(k,:),'b');
        %axis([angles(1),angles(nIts),plotMin,plotMax])
        axis auto;
        title(num2str(k))
        
        subAngs=angles(fitReg(1):fitReg(2));
        subX=allCogX(k,fitReg(1):fitReg(2));
        subY=allCogY(k,fitReg(1):fitReg(2));
        pX = polyfit(subAngs,subX,1);
        pY = polyfit(subAngs,subY,1);
        %subplot(4,4,k*2-1)
        plot(angles,angles*pX(1)+pX(2),'--r')
        %subplot(4,4,k*2)
        plot(angles,angles*pY(1)+pY(2),'--b')
        allXCoeffs(k,:)=pX;
        allYCoeffs(k,:)=pY;
        %axis([0,nIts,plotMin,plotMax])
        
    else
        plot(allCogX(k,:),'r');
        plot(allCogY(k,:),'b');
        axis([0,nIts,plotMin,plotMax])
        %axis auto
        title(num2str(k))
    end
    hold off
end

disp(['Average X coefficient 1: ' num2str(mean(allXCoeffs(:,1)))])
disp(['Average Y coefficient 1: ' num2str(mean(allYCoeffs(:,1)))])
disp(['Average X coefficient 2: ' num2str(mean(allXCoeffs(:,2)))])
disp(['Average Y coefficient 2: ' num2str(mean(allYCoeffs(:,2)))])


