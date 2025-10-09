function sendTrajUDP(traj, SimIn, headsetAddress, headsetPortHUD, headsetPortTraj)

% Send data over UDP
traj_flat = reshape(traj.', [], 1);  % 3N x 1 column vector
traj_bytes = typecast(double(traj_flat), 'uint8');  % 3N*8 bytes

% We will build UDP 512 bytes packets with a 8 byte header. The header will
% contain the following information in the following order:
% 0-1 Trajectory ID
% 2-3 Total number of packets
% 4-5 Packet number
% 6-7 Payload size in bytes
% 8-N Payload
packet_size = 512;
header_size = 8;
max_payload = packet_size - header_size;
num_packets = ceil(length(traj_bytes) / max_payload);

traj_id = uint16(1);

u = udpport("Datagram", "IPV4");

for k = 1:num_packets
    idx_start = (k-1)*max_payload + 1;
    idx_end = min(k*max_payload, length(traj_bytes));
    chunk = traj_bytes(idx_start:idx_end);
    
    header = [...
        typecast(uint16(traj_id), 'uint8'), ...
        typecast(uint16(num_packets), 'uint8'), ...
        typecast(uint16(k), 'uint8'), ...
        typecast(uint16(length(chunk)), 'uint8') ...
    ];
    
    msg = [header'; chunk];
    write(u, msg, "uint8", headsetAddress, headsetPortTraj);
    pause(0.01);  % avoid flooding
end
disp('Trajectory sent to headset');


% Send a default package to the HUD
rad2deg = 180/pi;
hudData = [SimIn.IC.phi*rad2deg, SimIn.IC.theta*rad2deg, SimIn.IC.psi*rad2deg,... 
    SimIn.EOM.Pos_bii(1)/SimIn.Units.m, SimIn.EOM.Pos_bii(2)/SimIn.Units.m, -SimIn.EOM.Pos_bii(3)/SimIn.Units.m,...
    SimIn.IC.LatGeod*rad2deg, SimIn.IC.Lon*rad2deg, 0,...
    SimIn.IC.Vtot/SimIn.Units.knot, 0, 0];

dataBytes = typecast(double(hudData), 'uint8');
write(u, dataBytes, 'uint8', headsetAddress, headsetPortHUD);

end