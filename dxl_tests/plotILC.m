
it = 0;

itStr = "Iteracao %d";
baseFileName = "K%d.txt";


while true
  input(sprintf(itStr,it));
  
  M = dlmread(sprintf(baseFileName,it));
  
  t     = M(:,1);
  qref  = M(:,2);
  qd    = M(:,3);
  q     = M(:,4);
  U     = M(:,5);
  err   = M(:,6);
  
  plot(t,qref,t,qd,t,q,t,U,t,err)
  legend('qref','qd','q','U','error')
  
  it = it + 1
end
