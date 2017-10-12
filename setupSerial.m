function [s,flag]=setupSerial(comPort)

flag=1;
s=serial(comport);

set(s,'DataBits',8);
set(s,'StopBits',1);
set(s,'BaudRate',9600);
set(s,'Parity','none');
