sharedMemKey = 'nullerRTSHM';
olddata = 0;
tic

robj = shmref(sharedMemKey);
    
while true
    
    newdata = robj.data(1);

    %newdata=rand(100,1);
    
    disp(['Newdata(1): ' num2str(newdata(1))])
    disp(['Olddata(1): ' num2str(olddata(1))])
    disp('')
    
     if newdata(1) ~= olddata(1)
         disp('Inner')
%         toc
         %disp(newdata);
         olddata = newdata;
         disp('Inner2')
%         tic
     end
    

    pause(0.1)
end
delete(robj);
    
