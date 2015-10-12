%imStoreCube=imCube;
imStoreCube=squeeze(allSubIms(:,:,1,:));

figure(1)
for ii = 1:10:1000
    imagesc(imStoreCube(:,:,ii))
    %disp(angles(ii))
    disp(ii)
    pause(0.05)
end

%figure(2)
%plot(squeeze(imStoreCube(50,50,:)))