function [position, fall, A, B , C] = triangulering(leftAng,rightAng,len)
%TRIANGULATION Summary of this function goes here
%   Detailed explanation goes here

    if leftAng <= 0 && rightAng >= 0
        A = 90 - abs(leftAng);
        B = 90 - abs(rightAng);
        C = 180 - A - B;
        if C <= 5
            %disp('sket sig')
            position = NaN;
            fall = NaN;
            A = NaN;
            B = NaN;
            C = NaN;            
            return
        end
        fall = 1;
        %HL = (len * sin(deg2rad(A))) / sin(deg2rad(C));
        VL = (len * sin(deg2rad(B))) / sin(deg2rad(C));

        %x1_r = HL * cos(deg2rad(B));
        
        %ypos_R = HL * sin(deg2rad(B));
        %xpos_R = len - x1_r;
        
        xpos_L = VL * cos(deg2rad(A));
        ypos_L = VL * sin(deg2rad(A));
        
        
    elseif leftAng > 0 && rightAng > 0
        A = 90 + abs(leftAng);
        B = 90 - abs(rightAng);
        C = 180 - A - B;
        if C <= 5
            %disp('sket sig')
            position = NaN;
            fall = NaN;
            A = NaN;
            B = NaN;
            C = NaN;            
            return
        end
        fall = 2;
        %HL = (len * sin(deg2rad(A))) / sin(deg2rad(C));
        VL = (len * sin(deg2rad(B))) / sin(deg2rad(C));
        
        %x1_r = HL * cos(deg2rad(B));
        
        %ypos_R = [ypos_R, HL * sin(deg2rad(B))];
        %xpos_R = [xpos_R, x1_r - len];
        
        xpos_L = -1 * VL * cos(deg2rad(180 - A));
        ypos_L = VL * sin(deg2rad(180 - A));
        
    elseif leftAng < 0 && rightAng < 0
        A = 90 - abs(leftAng);
        B = 90 + abs(rightAng);
        C = 180 - A - B;
        if C <= 5
            %disp('sket sig')
            position = NaN;
            fall = NaN;
            A = NaN;
            B = NaN;
            C = NaN;
            return
        end
        fall = 3;
        %HL = (len * sin(deg2rad(A))) / sin(deg2rad(C));
        VL = (len * sin(deg2rad(B))) / sin(deg2rad(C));
        
        %x1_r = HL * cos(deg2rad(180 - B));
        
        %ypos_R = HL * sin(deg2rad(180 - B));
        %xpos_R = x1_r + len;
        
        xpos_L = VL * cos(deg2rad(A));
        ypos_L = VL * sin(deg2rad(A));
    else
        disp('här');
    end
    
    position = [xpos_L,ypos_L];
end

