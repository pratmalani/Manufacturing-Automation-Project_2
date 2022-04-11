% Manufactuirng Automation
% Lab 3
close all; clear; clc

%% Connecting to V-rep 

disp('Program started'); %Declare start of program
vrep=remApi('remoteApi'); %Remote connection to API
vrep.simxFinish(-1);  %Close all other ports and connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5); %Start V-rep simulation 
% [returnCode] = vrep.simxsynchronous(clientID,1);
% vrep.simxStart(19999,vrep.simx_opmode_blocking);


%% Knocking off all of the objects with the linear actuator

if (clientID>-1)
    disp('Connected to remote API server');
        
       

%% Add your code here
        [returnCode1, visionsenor] = vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking);
        [returnCode_j, prismaticjoint] = vrep.simxGetObjectHandle(clientID,'Prismatic_joint',vrep.simx_opmode_blocking);
       
        t=0;
        tic;
        
        while t<30
            t=toc;

            [returnCode_v1,resolution,Iv]=vrep.simxGetVisionSensorImage2(clientID,visionsenor,0, vrep.simx_opmode_streaming);
 
            I = Iv;
            I2 = double(I);
            if max(I(:)) == 255
            I1 = rgb2gray(I);
            Id = double(I1);
            [Gx,Gy] = gradient(Id); % gradient
            M = sqrt(Gx.^2+Gy.^2);  
            M = M./max(M(:)); 
            Ia = M<0.02; 
    
            conn = bwconncomp(Ia,8);
            nobj = conn.NumObjects;
            L = zeros(1,nobj);
            
            for i = 1:nobj
                L(i) = length(conn.PixelIdxList{i});
            end

                object_id = find(L>100);
                nShapes=length(object_id);
                 x = 1:size(Ia,2);           y = 1:size(Ia,1);
                [xx,yy] = meshgrid(x,y);    xx = xx(:);     yy=yy(:); 
                for i=1:nShapes
                BW = 0*Ia;
                shapeID = conn.PixelIdxList{object_id};
                BW(shapeID) = 1;
                BW = reshape(BW,size(Ia,1),size(Ia,2));
                A = sum(BW);      w_image = sum(A>0);      
                B = sum(BW,2);    h_image = sum(B>0); 
                 
                 xobj{i}=xx(shapeID);    yobj{i}=yy(shapeID);
                 xbb{i} = [min(xobj{i}) max(xobj{i}) max(xobj{i}) min(xobj{i}) min(xobj{i})];
                 ybb{i} = [min(yobj{i}) min(yobj{i}) max(yobj{i}) max(yobj{i}) min(yobj{i})];
              
                    
                  if max(max(I2(:,:,2)))>200 &&  w_image<20 %%This gives rejection for colour and size, could not figure out for shape.Tried comparing the area of shape and boundaries of shape.
                vrep.simxSetJointPosition( clientID,prismaticjoint,0.2, vrep.simx_opmode_blocking);

                vrep.simxSetJointPosition( clientID,prismaticjoint ,0, vrep.simx_opmode_blocking);
                  end
                  
                end
            
     end
        end

end

        