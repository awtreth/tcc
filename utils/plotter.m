
filepath = '/tmp/gzLog.txt';
prevtime = dir(filepath).datenum;
figure;
t=(0:99)';

%loopcounter = 0;
while true
  %currtime = dir(filepath).datenum;
  %if currtime > prevtime
  %  prevtime = currtime;
    % see http://octave.sourceforge.net/octave/function/textread.html
    [x, y] = textread(filepath, '%f %f');
    
    if(rows(x) < 100)
      x = [zeros(100-rows(x),1);x];
      y = [zeros(100-rows(y),1);y];
    end  
    
    plot(t,x(end-99:end),'r-', t, y(end-99:end), 'g-');
    axis ([0 100 -1 1])
  %end
  pause(0.1);
end