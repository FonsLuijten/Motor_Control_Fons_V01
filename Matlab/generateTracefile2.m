file = 'C:\Users\fonsl\OneDrive\Projecten\Mavilor motors\Software\Motor_Control_Fons_V01\Arduino\MotorControlArduino\defines.h';
file = 'D:\OneDrive\Projecten\Mavilor motors\Software\Motor_Control_Fons_V01\Arduino\MotorControlArduino\defines.h';

h = fopen(file);
text = fscanf( h , '%c');
fclose( h );

textsplit = strsplit( text , {' ' , ';' , '\n' });
textsplit( find(contains(textsplit , '[]'))-1) = [];

float = {};
uint = {};
int = {};
bool = {};
floatindex = [];
uintindex = [];
intindex = [];
boolindex = [];
floatsize = {};
uintsize = {};
intsize = {};
boolsize = {};


for i = 1:length(textsplit)-1
    if(strcmp(textsplit{i} , 'float' )||strcmp(textsplit{i} , 'int' )||strcmp(textsplit{i} , 'bool' ))
        varsplit = strsplit( textsplit{i+1} , {' ','[',']'});
        if length(varsplit)>3
            varsize = {[varsplit{2} ';' varsplit{3}]};
        elseif length(varsplit)>2
            varsize = {[varsplit{2} ';0']};
        elseif length(varsplit)>1
            error('Check size format of variable')
        elseif length(varsplit)==1
            varsize = {['0;0']};
        else
            error('Check size format of variable')
        end
        
        if max(isnan(str2double(strsplit(varsize{:},';'))))
            error('Arrays must be declared as xxxx[nrows][ncols] = ....')
        end
        varsizenum = str2double(strsplit(varsize{:},';')); 
        var = varsplit{1};    
        
        if varsizenum(1)>0 && varsizenum(2)>0
            varorg = var;
            varsizeorg = varsize{:};
            var = {};
            varsize = {};
            for jj = 1:varsizenum(2)
                for ii = 1:varsizenum(1)
                    var = [var [varorg '[' num2str(ii-1) ']' '[' num2str(jj-1) ']']];
                    varsize = [varsize varsizeorg];
                end
            end
        elseif varsizenum(1)>0
            varorg = var;
            varsizeorg = varsize{:};
            var = {};
            varsize = {};
            for ii = 1:varsizenum(1)
                var = [var [varorg '[' num2str(ii-1) ']']];
                varsize = [varsize varsizeorg];
            end
        else
            var = {var};
        end
    end
        
    if(strcmp(textsplit{i} , 'float' ))
        float = [float var];
        floatsize = [floatsize varsize];
        if(strcmp(textsplit{i-1} ,'const'))
            floatindex(length(float)) = 1;
        else
            floatindex(length(float)) = 0;
        end
    end
    if(strcmp(textsplit{i} , 'int' ))
        if( i-1 > 1 && strcmp(textsplit{i-1} , 'unsigned' )) 
            uint = [uint var];
            uintsize = [uintsize varsize];
            if(strcmp(textsplit{i-2} ,'const'))
                uintindex(length(uint)) = 1;
            else
                uintindex(length(uint)) = 0;
            end
        else
            int = [int var];
            intsize = [intsize varsize];
            if(strcmp(textsplit{i-1} ,'const'))
                intindex(length(int)) = 1;
            else
                intindex(length(int)) = 0;
            end
        end
    end
    if(strcmp(textsplit{i} , 'bool' ))
        bool = [bool var];
        boolsize = [boolsize varsize];
        if(strcmp(textsplit{i-1} ,'const'))
            boolindex(length(bool)) = 1;
        else
            boolindex(length(bool)) = 0;
        end
    end
end


%% Write txt file
txt = { 'void trace( ) {'};
txt = [txt '  for( int i = 0; i<10; i++){'];
txt = [txt '    int isignal = tracearray[i];'];
jsignal = 0;
txt = [txt ['    switch( isignal ){' ]];
for i=1:length(float)
    txtadd = ['      case ' sprintf( '%3i' ,jsignal) ': bf.fp   = ' float{i} '; break;'];
    txt = [txt txtadd];
    jsignal = jsignal + 1;
    signaltype( jsignal ) = 'f';
end
for i=1:length(int)
    txtadd = ['      case ' sprintf( '%3i' ,jsignal) ': bf.sint = ' int{i} '; break;'];
    txt = [txt txtadd];
    jsignal = jsignal + 1;
    signaltype( jsignal ) = 'i';
end
for i=1:length(uint)
    txtadd = ['      case ' sprintf( '%3i' ,jsignal) ': bf.uint = ' uint{i} '; break;'];
    txt = [txt txtadd];
    jsignal = jsignal + 1;
    signaltype( jsignal ) = 'I';
end
for i=1:length(bool)
    txtadd = ['      case ' sprintf( '%3i' ,jsignal) ': bf.bl   = ' bool{i} '; break;'];
    txt = [txt txtadd];
    jsignal = jsignal + 1;
    signaltype( jsignal ) = 'b';
end

txt = [txt ['    }' ]];
txt = [txt  '    Serial.write( bf.bin , 4);'];
txt = [txt '  }'];
txt = [txt ['}' newline]];

%% 
txt = [txt 'void setpar( int isignal , binaryFloat bf ) {'];
jsignal = 0;
txt = [txt ['  switch( isignal ){' ]];
for i=1:length(float)
    if( floatindex(i) == 0 )
        txt = [txt ['    case ' sprintf( '%3i' ,jsignal) ': ' float{i} ' = bf.fp; break;']];
    end  
    jsignal = jsignal + 1;
end
for i=1:length(int)
    if( intindex(i) == 0 )
        txt = [txt ['    case ' sprintf( '%3i' ,jsignal) ': ' int{i} ' = bf.sint; break;']];
    end
    jsignal = jsignal + 1;
end
for i=1:length(uint)
    if( uintindex(i) == 0 )
        txt = [txt ['    case ' sprintf( '%3i' ,jsignal) ': ' uint{i} ' = bf.uint; break;']];
    end
    jsignal = jsignal + 1;
end
for i=1:length(bool)
    if( boolindex(i) == 0 )
        txt = [txt ['    case ' sprintf( '%3i' ,jsignal) ': ' bool{i} ' = bf.bl; break;']];
    end
    jsignal = jsignal + 1;
end

txt = [txt '  }'];
txt = [txt ['}' newline]];

%%

txt = [txt 'void printSignals( unsigned int selected ) {'];
txt = [txt ['  char *signalNames[] = { ' sprintf( '"%s", ' , float{:} , int{:} , uint{:}  , bool{:} ) ' };']];
txt = [txt ['  char *signalTypes[] = { ' sprintf( '"%c", ' , signaltype ) ' };']];
txt = [txt ['  char *signalSizes[] = { ' sprintf( '"%s", ' , floatsize{:} , intsize{:} , uintsize{:}  , boolsize{:} ) ' };']];

txt = [txt '  int imax = 10;'];
txt = [txt '  switch(selected){'];
txt = [txt ['    case 0: imax = ' int2str(jsignal) '; break;' ]];
txt = [txt '  }'];

%%
txt = [txt '  for ( int i = 0; i < imax; i++) {'];
txt = [txt '    Serial.println( signalNames[i] );'];
txt = [txt '    Serial.println( signalTypes[i] );'];
txt = [txt '    Serial.println( signalSizes[i] );'];
txt = [txt '  }'];
txt = [txt '  Nsend = 0;'];
txt = [txt '  getsignalnames = 1;'];
txt = [txt '}'];

  
% Write to file
fid = fopen('test.c','w');
for kLine = 1 : length(txt)
    fprintf(fid, '%s\n', txt{kLine});
end
fclose all ;
disp('finished')

%%
% return;
% % Write to ino file
% idx = strfind(file,'\');
% inoname = [file((idx(end-1)+1):(idx(end)-1)) '.ino'];
% fileino = strrep(file,'defines.h',inoname);
% 
% copyfile(fileino,strrep(fileino,'.ino','.txt'))
% fid = fopen(fileino,'r');
% idx = fseek(fid, 0, 'void trace( ) {');
% fclose all ;
% 
% for kLine = 1 : length(txt)
%     fprintf(fid, '%s\n', txt{kLine});
% end
% fclose all ;
% disp('finished')