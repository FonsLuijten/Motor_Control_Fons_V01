[a,b] = audioread('C:\Users\Fons Luijten\Downloads\He''s a Pirate.wav');

disp('Done reading audio file')

maxlength = 1e5;

totallength = min([maxlength length(a)]);
clear txt
txt = {'const float muziek[' num2str(totallength) '] = {' newline};
for i = 1:totallength
    txt = [txt num2str(a(i,1)) ',' newline];
    disp(i)
end
txt = [txt '};' newline];

disp('Done making text')

% Write to file
fid = fopen('muziek.c','w');
for kLine = 1 : length(txt)
    fprintf(fid, '%s\n', txt{kLine});
end
fclose all ;
disp('finished')

disp('Done writing to file')