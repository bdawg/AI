
clear all

reSamp = 50;
tStep=2;
plotMin=-4;
plotMax=4;
showAnimation = true;

%load('data/aiDataSave_20151015T165804')
%load('data/aiDataSave_20150930T145344')
%load('data/aiDataSave_20151001T171113')
%load('data/aiDataSave_20151001T175859_set1_01')

load('data/aiDataSave_20151125T030431')

nVals = size(allCogXs,2);
nSubs = size(allCogXs,1);
reSampInds = 1:reSamp:nVals;
allSubImsMultiOpen = allSubImsMulti(:,:,:,reSampInds);
allCogXsOpen = allCogXs(:,reSampInds);
allCogYsOpen = allCogYs(:,reSampInds);
clear allSubImsMulti
clear allCogXs
clear allCogYs
nVals = size(allCogXsOpen,2);

%load('data/aiDataSave_20151015T165804')
load('data/aiDataSave_20151125T030431.mat')
%load('data/aiDataSave_20151001T175951_set1_02')
allSubImsMulti = allSubImsMulti(:,:,:,reSampInds);
allCogXs = allCogXs(:,reSampInds);
allCogYs = allCogYs(:,reSampInds);
nVals = size(allCogXs,2);
if showAnimation
    figure(1)
    for t = 1:tStep:nVals
        for kk = 1:nSubs
            subplot(4,2,kk)
            imagesc(allSubImsMultiOpen(:,:,kk,t))
            %axis equal
        end
        drawnow
    end

    for t = 1:tStep:nVals
        for kk = 1:nSubs
            subplot(4,2,kk)
            imagesc(allSubImsMulti(:,:,kk,t))
            %axis equal
        end
        drawnow
    end
end

figure(2)
clf
allCogXsCombined=[allCogXsOpen allCogXs];
allCogYsCombined=[allCogYsOpen allCogYs];
%allCogXsCombined=allCogXsOpen;
%allCogYsCombined=allCogYsOpen;

for kk = 1:nSubs
    subplot(4,2,kk)
    hold on
    plot(allCogXsCombined(kk,:),'r');
    plot(allCogYsCombined(kk,:),'b');
    axis([0,nVals*2,plotMin,plotMax])
    %axis auto
    title(num2str(kk))
    hold off
end


