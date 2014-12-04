%% MEAM410 Localization
clc; clear; close all;

smoothie = false; %add smoothing
spin = true; %add spin theta
vec_scale = 2;

load C
% load V
% load C
 
data = rawStarData;
% data(data == 1023) =0; %set any data points of 1023 to 0
% %the previous line is wrong. It should like, not consider 1023 lines at
% %all. Still working on that implementation

% p1 = [data(1,1) data(1,5)];
% p2 = [data(1,2) data(1,6)];
% p3 = [data(1,3) data(1,7)];
% p4 = [data(1,4) data(1,8)]; 

%extract data to meaningful variable names
x1 = data(:,1);
x2 = data(:,2);
x3 = data(:,3);
x4 = data(:,4);

y1 = data(:,5);
y2 = data(:,6);
y3 = data(:,7);
y4 = data(:,8);

%find the distance between each of the points
d12 = sqrt( (x2 - x1).^2 + (y2 - y1).^2);
d13 = sqrt( (x3 - x1).^2 + (y3 - y1).^2);
d14 = sqrt( (x4 - x1).^2 + (y4 - y1).^2);
d23 = sqrt( (x3 - x2).^2 + (y3 - y2).^2);
d24 = sqrt( (x4 - x2).^2 + (y4 - y2).^2);
d34 = sqrt( (x4 - x3).^2 + (y4 - y3).^2);

%if the incoming data was 1023 then set that distance to 0. Not entirely
%sure if this works the way I think it does. No. Columns don't align. don't
%do that
% d(data == 1023) = 0; 

q = nan;
% take out the distances that came from 1023 data points properly
% the final position images look much different (better?) without this
% if Load A then this creates a large theta discontinuity. not sure why.
for i = 1:length(data)
    if((x1(i) == 1023) || (y1(i) == 1023))
       d12(i) = q;
       d13(i) = q;
       d14(i) = q;
    end
    if (x2(i) == 1023 || y2(i) == 1023)
       d12(i) = q;
       d23(i) = q;
       d24(i) = q;
    end
    if (x3(i) == 1023 || y3(i) == 1023)
        d13(i) = q;
        d23(i) = q;
        d34(i) = q;
    end
    if (x4(i) == 1023 || y4(i) == 1023)
       d14(i) = q;
       d24(i) = q;
       d34(i) = q;
    end
end
%check this out
% for k = 1:length(data);
%     if data(k,1) == 1023;
%         data(k,1) = NaN;
%     elseif data(k,2) == 1023;
%         data(k,2) = NaN;
%     elseif data(k,3) == 1023;
%         data(k,3) = NaN;
%     else data(k,4) = NaN;
%     end
% end

d = [d12 d13 d14 d23 d24 d34]; %create an array of the distances

[maxvalues, maxindex] = max(d,[],2);%find max values and their indexes

thetas = zeros(length(maxindex),1);%initialize thetas column array
centers = zeros(length(maxindex),2);%init centers column array
finals = zeros(length(maxindex),2);%init final positions to plot

up = [0 1]; %the y axis for cross product
first = true;

for i = 1:length(maxindex)
   %default to positive sign
   %psuedocode for every statement below
   %switch (each index of the maximum values)
   %    case(point A and point B are the furthest apart)
   %        vect = the vector between point A and B
   %        cent = the average point between the endpoints. This is the
   %               current center position.
   %        If(point A is closer to either of the currently unused points)
   %            if(Point B is further to the right than point A)
   %                then the rotation is clockwise, so the sign should be
   %                negative. The sign is positive otherwise.
   %
   %throughout, dBA == dAB so they're sometimes swapped for convenience
    heresyoursign = 1;
    
   switch maxindex(i)
       case 1 %d12           
           %another conditional for when all hte columns are NaN
           %just keep the center the same if you can't tell MATLAB jesus
           if not((x1(i) == 1023) || (y1(i) == 1023) || x2(i) == 2013 || y2(i) == 1023)
                cent = [ (x1(i) + x2(i)) , (y1(i) + y2(i)) ]./2;
           end
                      
           if(d23(i) < d13(i) || d24(i) < d14(i)) %if 2 is the top
               vect = [ (x2(i) - x1(i)) , (y2(i) - y1(i)) ]; %from bottom to top... top minus bottom
               if (x2(i) < x1(i))
                   heresyoursign = -1;
               end
           else
               %then 1 is the top and...
               vect = [ (x1(i) - x2(i)) , (y1(i) - y2(i)) ];
               if (x1(i) < x2(i))
                   heresyoursign = -1;
               end
           end
       case 2 %d13
           cent = [ (x1(i) + x3(i)) , (y1(i) + y3(i)) ]./2;
           
           if(d14(i) < d34(i) || d12(i) < d23(i)) %1 is top
               vect = [ (x1(i) - x3(i)) , (y1(i) - y3(i)) ];
               if (x1(i) < x3(i))
                   heresyoursign = -1;
               end
           else%3 is top
               vect = [ (x3(i) - x1(i)) , (y3(i) - y1(i)) ];
               if (x3(i) < x1(i))
                   heresyoursign = -1;
               end
           end
           
       case 3 %d14
           cent = [ (x1(i) + x4(i)) , (y1(i) + y4(i)) ]./2;
           
           if(d12(i) < d24(i) || d13(i) < d34(i)) %1 is the top
               vect = [ (x1(i) - x4(i)) , (y1(i) - y4(i)) ];
               if (x1(i) < x4(i))
                   heresyoursign = -1;
               end
           else %4 is top
               vect = [ (x4(i) - x1(i)) , (y4(i) - y1(i)) ];
               if (x4(i) < x1(i))
                   heresyoursign = -1;
               end
           end
       case 4 %d23
           cent = [ (x2(i) + x3(i)) , (y2(i) + y3(i)) ]./2;
           
           if(d12(i) < d13(i) || d24(i) < d34(i))%2 is top
               vect = [ (x2(i) - x3(i)) , (y2(i) - y3(i)) ];
               if (x2(i) < x3(i))
                   heresyoursign = -1;
               end
           else
               vect = [ (x3(i) - x2(i)) , (y3(i) - y2(i)) ];
               if (x3(i) < x2(i))
                   heresyoursign = -1;
               end
           end
       case 5 %d24
           cent = [ (x2(i) + x4(i)) , (y2(i) + y4(i)) ]./2;
           
           if(d12(i) < d14(i) || d23(i) < d34(i))%2 is top
               vect = [ (x2(i) - x4(i)) , (y2(i) - y4(i)) ];
               if (x2(i) < x4(i))
                   heresyoursign = -1;
               end
           else%4 is top
               vect = [ (x4(i) - x2(i)) , (y4(i) - y2(i)) ];
               if (x4(i) < x2(i))
                   heresyoursign = -1;
               end 
           end
       case 6 %d34
           cent = [ (x3(i) + x4(i)) , (y3(i) + y4(i)) ]./2;
           
           if(d13(i) < d14(i) || d23(i) < d24(i)) %3 is top
               vect = [ (x3(i) - x4(i)) , (y3(i) - y4(i)) ];
               if (x3(i) < x4(i))
                   heresyoursign = -1;
               end
           else
               vect = [ (x4(i) - x3(i)) , (y4(i) - y3(i)) ];
               if (x4(i) < x3(i))
                   heresyoursign = -1;
               end
           end
       otherwise 
           disp('maxindex range exceeded')
   end
   
   centers(i,1) = cent(1);
   centers(i,2) = cent(2);
   
   thetas(i) = heresyoursign*acos( dot(up, (vect/norm(vect))) );
   
%    % control for outliers
%    if( not(first) && (thetas(i) > thetas(i-1) + 10))
%        thetas(i) = thetas(i -1);
%    end
   
%    if( not(first) && centers(i,1) > (centers(i-1,1) + 100))
%         centers(i,1) = centers(i-1,1);
%    end
%    
%    if( not(first) && centers(i,2) > (centers(i-1,2) + 100))
%         centers(i,2) = centers(i-1,2);
%    end
   
   first = false;    
end

   if(smoothie)
       thetas = smooth(thetas,8); %8 is the number of points to consider in the moving average
       centers(:,1) = smooth(centers(:,1));
       centers(:,2) = smooth(centers(:,2));
   end
   
%do the rotation if the spin variable is set
for i = 1:length(maxindex)
   if(spin)
       R = [cos(thetas(i)) -sin(thetas(i)); ...
            sin(thetas(i))  cos(thetas(i))];
       pvect = [(1024/2 - centers(i,1)), (768/2 - centers(i,2))]; %vector from rink center to camera
       %R = [cosd(90) -sind(90); sind(90) cosd(90)];
       %interim = R*pvect'; %position of the robot with (0,0) origin at the rink center
       Rot = [ cos(thetas(i)) -sin(thetas(i)) 0; sin(thetas(i)) cos(thetas(i)) 0; 0 0 1];
       T = [ 1 0 centers(i,1); 0 1 centers(i,2); 0 0 1];
       Tn = T;
       Tn(1,3) = -Tn(1,3);
       Tn(2,3) = -Tn(2,3); 
       c = [pvect 1];
       %c = [pvect 1];
       H = T * Rot * Tn* c';
       interim = H;
   else
       R = [1 0; 0 1];
       interim = R*[centers(i,:)]'; 
   end
   
   finals(i,1) = interim(1);
   finals(i,2) = interim(2);
end

%vectors for quiver plot
u = sin(-thetas);
v = cos(-thetas);

t = 1:length(thetas); %dummy variable just to see the values of thetas

if(smoothie)
    %some final smoothing. lol puns.
    finals(:,1) = smooth(finals(:,1));
    finals(:,2) = smooth(finals(:,2));
end

plot(finals(:,1), finals(:,2), 'o');
hold on
% plot(finals(:,1), finals(:,2), '*c');
title('positions')

axis([-1024/2 1024/2 -768/2 768/2]);
axis equal
% figure(2)
quiver(finals(:,1), finals(:,2),u,v,vec_scale);
% figure(2)
% plot(t,thetas)
% title('thetas as a function of time')%time-ish, time scale is whatever the sampling period was)