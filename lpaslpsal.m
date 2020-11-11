
function [SP1,SP2]=squarelineintercept(vRTx,vRTy,vRTmaxx,vRTmaxy,vax,vay,vbx,vby,vcx,vcy,vdx,vdy)

mRS=slope(vRTx,vRTy,vRTmaxx,vRTmaxy);
cRS=yintercept(vRTx,vRTy,vRTmaxx,vRTmaxy);

mab=slope(vax,vay,vbx,vby);
cab=yintercept(vax,vay,vbx,vby);

mbc=slope(vbx,vby,vcx,vcy);
cbc=yintercept(vbx,vby,vcx,vcy);

mcd=slope(vcx,vcy,vdx,vdy);
ccd=yintercept(vcx,vcy,vdx,vdy);

mad=slope(vax,vay,vdx,vdy);
cad=yintercept(vax,vay,vdx,vdy);

SP1=[];
SP2=[];

if(mab==mRS)
    
    display('No line intersection');
    
else
 
      F=intersectline(vax,vay,vbx,vby,vRTx,vRTy,vRTmaxx,vRTmaxy);
      
      x=F(1);
      y=F(2);
      
      [in,on]=inpolygon(x,y,FVRX,FVRY);
      
      if(in==1 && on==1)
          
          TF1=isempty(SP1);
          TF2=isempty(SP2);
          
          if(TF1==1)
              
              SP1=F
          else
              SP2=F
          end
      else
          display('intersection outside the FVR');
      end
end

if(TF1==1 || TF==1)
    
    if(mbc==mRS)
        
        display('No line intersection');
       
    else
        
         F=intersectline(vbx,vby,vcx,vcy,vRTx,vRTy,vRTmaxx,vRTmaxy);
      
      x=F(1);
      y=F(2);
      
      [in,on]=inpolygon(x,y,FVRX,FVRY);
      
      if(in==1 && on==1)
          
          TF1=isempty(SP1);
          TF2=isempty(SP2);
          
          if(TF1==1)
              
              SP1=F
          else
              SP2=F
          end
      else
            display('intersection outside the FVR');
            
      end
    end
else
end

if(TF1==1 || TF==1)
    
    if(mcd==mRS)
        
        display('No line intersection');
       
    else
        
         F=intersectline(vcx,vcy,vdx,vdy,vRTx,vRTy,vRTmaxx,vRTmaxy);
      
      x=F(1);
      y=F(2);
      
      [in,on]=inpolygon(x,y,FVRX,FVRY);
      
      if(in==1 && on==1)
          
          TF1=isempty(SP1);
          TF2=isempty(SP2);
          
          if(TF1==1)
              
              SP1=F
          else
              SP2=F
          end
      else
            display('intersection outside the FVR');
            
      end
    end
else
end

if(TF1==1 || TF==1)
    
    if(mad==mRS)
        
        display('No line intersection');
       
    else
        
         F=intersectline(vdx,vdy,vax,vay,vRTx,vRTy,vRTmaxx,vRTmaxy);
      
      x=F(1);
      y=F(2);
      
      [in,on]=inpolygon(x,y,FVRX,FVRY);
      
      if(in==1 && on==1)
          
          TF1=isempty(SP1);
          TF2=isempty(SP2);
          
          if(TF1==1)
              
              SP1=F
          else
              SP2=F
          end
      else
            display('intersection outside the FVR');
            
      end
    end
else
end

end