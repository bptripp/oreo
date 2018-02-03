function [] = udp_test()

pnet('closeall');

sock=pnet('udpsocket',1700);

pnet(sock,'setwritetimeout',1);
pnet(sock,'setreadtimeout',0);

for i=1:6000
pnet(sock, 'write', uint8([1,3,1,9,1]));
pnet(sock,'writepacket', '127.0.0.1',1700);
end

size=pnet(sock,'readpacket','noblock');
count = 0;
tic;
while(size > 0)
    data=pnet(sock,'read', size,'uint8');
    count = count+1;
    size=pnet(sock,'readpacket','noblock');
end
time = toc;
disp(sprintf('Time: %f Count: %f' , time, count));
end 