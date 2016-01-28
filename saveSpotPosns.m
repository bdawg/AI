% A script to save the nominal centre positions of each spot
% full image should be in 'im'


nSpots=8;
imsz=128;
winSize = 16

imagesc(im);
disp('Click approximate positions of each spot, then press Enter.');
[clickX, clickY] = ginput(nSpots);
bias=min(im(:));
winCents=zeros(nSpots,2);

for ii = 1:nSpots
    curIm=double(im-bias);
    mask=zeros(imsz,imsz);
    mask(clickY(ii)-winSize/2:clickY(ii)+winSize/2-1, ...
            clickX(ii)-winSize/2:clickX(ii)+winSize/2-1) = 1;
    curIm = curIm .* mask;
    hold on
    imagesc(curIm);
    [maxval, idx] = max(curIm(:));
    [yPosn, xPosn] = ind2sub(size(curIm), idx);
    plot(xPosn, yPosn, 'r*');
    winCents(ii,:) = [yPosn, xPosn];
    hold off
    
    disp(ii)
    pause(1)
end

    
save('winCents.mat', 'winCents');
save(['winCents_' datestr(now,30) '.mat'], 'winCents');