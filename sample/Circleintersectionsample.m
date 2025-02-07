clc 
clear

%coordinates of joints
A=[7 4 0];
B=[5 16 0];
E=[18 35 0];
C=[25 25 0];
D=[23 10 0];
F=[43 32 0];
G=[45 17 0];

%length of each link/ distance between joints
AB=norm(B-A);
BC=norm(C-B);
CD=norm(D-C);
CE=norm(E-C);
BE=norm(E-B);
EF=norm(F-E);
FG=norm(G-F);


%now obtain the new positions 
%initial angle of link AB
initialAngle_AB=atan2(B(2)-A(2),B(1)-A(1)); 

%if this is negative, then we should subtract as shown below

if(initialAngle_AB<0)
angleAB_horizontal=2*pi+initialAngle_AB; %adjusting the angle to be in the ccw direction from the hozizontal
else 
    angleAB_horizontal = initialAngle_AB;
end 

%angleAB_horizontal is the angle that link AB makes with respect to x axis.
%

    
for theta=0:1:360
    
%position analysis

%increase angle by 1 deg

%new position of B
B_new = vpa(A+[AB*cos(angleAB_horizontal+deg2rad(theta)) AB*sin(angleAB_horizontal+deg2rad(theta)) 0]);

%new position of C

[C_x,C_y]=circcirc(B_new(1),B_new(2),BE,D(1),D(2),CD);

%checking if circles are not intersecting

circIntersect_x = any(isnan(vpa(C_x))); %checking for Not-a-Number
circIntersect_y = any(isnan(vpa(C_y)));


if circIntersect_x==0 && circIntersect_y==0 % if the circles are intersecting
    C_1=[C_x(1) C_y(1) 0]; %adding the two solutions
    C_2=[C_x(2) C_y(2) 0];
    dist1 = norm(C_1-C);
    dist2 = norm(C_2-C);

    if(dist1<dist2) %checking which new C is closer
      C_new=vpa(C_1);
    else
      C_new=vpa(C_2);
    end
    
    %new position of E
    
    [E_x,E_y]=circcirc(B_new(1),B_new(2),BE,C_new(1),C_new(2),CE);
    
    %checking if circles are not intersecting

    circIntersect_x = any(isnan(vpa(E_x))); %checking for Not-a-Number
    circIntersect_y = any(isnan(vpa(E_y)));


    if circIntersect_x==0 && circIntersect_y==0 % if the circles are  intersecting
         E_1=[E_x(1) E_y(1) 0]; %adding the two solutions
         E_2=[E_x(2) E_y(2) 0];
         dist1 = norm(E_1-E);
         dist2 = norm(E_2-E);

        if(dist1<dist2)
            E_new=vpa(E_1);
        else
            E_new=vpa(E_2);
        end
    
    % new position of F
    
        [F_x,F_y]=circcirc(E_new(1),E_new(2),EF,G(1),G(2),FG);
        
          %checking if circles are not intersecting

        circIntersect_x = any(isnan(vpa(F_x))); %checking for Not-a-Number
        circIntersect_y = any(isnan(vpa(F_y)));

        if circIntersect_x==0 && circIntersect_y==0  % if the circles are  intersecting
              F_1=[F_x(1) F_y(1) 0]; %adding the two solutions
              F_2=[F_x(2) F_y(2) 0];
              dist1 = norm(F_1-F);
              dist2 = norm(F_2-F);

              if(dist1<dist2)
                   F_new=vpa(F_1);
              else
                   F_new=vpa(F_2);
              end
              
              
              %storing values
              newB_x(theta+1)=B_new(1);
              newB_y(theta+1)=B_new(2);
              newC_x(theta+1)=C_new(1);
              newC_y(theta+1)=C_new(2);
              newE_x(theta+1)=E_new(1);
              newE_y(theta+1)=E_new(2);
              newF_x(theta+1)=F_new(1);
              newF_y(theta+1)=F_new(2);

           
              
                  %saving this into an Excel spreadsheet
                
            positionsMatrix = [B_new C_new E_new F_new];
 
%             if (theta==0)
%            
%             dlmwrite('PositionsAndForceDiffPos.xls',positionsMatrix,'delimiter','\t','precision',4);
%             else
%             dlmwrite('PositionsAndForceDiffPos.xls',positionsMatrix,'-append',...
%             'delimiter','\t','precision',4);
%              end

        else 
           
              fprintf('New position F cannot be determined at this angle from the initial: %d',theta);
              break; 
        end 
    
    else 
     fprintf('New position E cannot be determined at this angle  from the initial: %d',theta);
    break; 
    end 
    
    
    
else 
    fprintf('New position C cannot be determined at this angle: %d',theta);
    break; 
end 

%end of position analysis
B=B_new;
C=C_new;
E=E_new;
F=F_new;

end
 figure
             
              ax1= subplot(2,2,1);
              plot(newB_x,newB_y);
              title(ax1,'Joint B')
              ax2=  subplot(2,2,2);
               plot(newC_x,newC_y);
                title(ax2,'Joint C')
               ax3=  subplot(2,2,3);
              plot(newE_x,newE_y);
                title(ax3,'Joint E')
             ax4=  subplot(2,2,4);
              plot(newF_x,newF_y);
                title(ax4,'Joint F')
%                 

