 %  if dSP1vRT+dSP1vRTmax==dvRTvRTmax
       
 %      disp('SP1 lies inside seg RS')
       
 %  else
       
 %      disp('SP1 lies outside seg RS')
 %      SP1=[];
      
 %  end
   
   if dvRTSP2vRTmax==dvRTvRTmax
       
       disp('SP2 lies inside seg RS')
       
   else
       
       disp('SP2 lies outside seg RS')
       SP2=[];
       
   end
   
 %  if ~isempty(SP1)==1 && ~isempty(SP2)==1
       
 %      disp('SP1 & SP2 lies inside seg RS')
       
 %      if dSP1vRT < dSP2vRT
           
 %          disp('SP2 is selected being bigger than SP1')
 %          SP1=[];
           
 %      else
 %          disp('SP1 is selected being bigger than SP1')
 %          SP2=[];
           
 %      end
       
 %  else
 %     if isempty(SP1)==1 && isempty(SP2)==1
           
 %          disp('vRTmax is the final velocity')
           
 %      else
 %      end
 %  end
   