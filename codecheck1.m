%    magAP1=sqrt((AP1(1))^2+(AP1(2))^2);
   %    magAP2=sqrt((AP2(1))^2+(AP2(2))^2);
   %    
   %    if magvRT <= magAP1 <= magvRTmax || magvRT <= magAP2 <= magvRTmax
   %        
   %        if magvRT <= magAP1 <= magvRTmax
   %            
   %            %AP1=AP1
   %            
   %        else
   %            
   %            AP1=[]
   %            
   %        end
   %        
   %        if  magvRT <= magAP2 <= magvRTmax
   %            
   %            %AP2=AP2
   %            
   %        else
   %            
   %            AP2=[]
   %            
   %        end
   %        
   %        if isempty(AP2)==0 && isempty(AP1)==0
   %            
   %            if magAP1<=magAP2
   %                
   %                Vc=AP2
   %                
   %            else
   %                
   %                Vc=AP1
   %                
   %            end
   %        else
   %            
   %            if AP1==AP1
   %                
   %                if magAP1<=magvRTmax 
   %                    
   %                   Vc=AP1
   %                   
   %                else
   %                    
   %                    Vc=vRTmax
   %                end
   %            else
   %               if magAP2<=magvRTmax
   %                   
   %                   Vc=AP2
   %                  
   %               else
   %                   Vc=vRTmax
   %                   
   %              end
   %            end
   %        end
   %    end