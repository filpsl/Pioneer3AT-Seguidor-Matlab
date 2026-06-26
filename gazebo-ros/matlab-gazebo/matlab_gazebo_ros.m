% CRIANDO O PUBLICADOR EM "/cmd_vel"

% Nó local 
cmdNode = ros2node("/cmdNode");

% Se alistando como publicador em '/cmd_vel'
cmdPub = ros2publisher(cmdNode,"/cmd_vel", "geometry_msgs/Twist");

% Configurando mensagem a ser enviada
cmdVels = ros2message(cmdPub);
cmdVels.linear.x = 0.0;
cmdVels.linear.y = 0.0;
cmdVels.linear.z = 0.0;
cmdVels.angular.x = 0.0;
cmdVels.angular.y = 0.0;
cmdVels.angular.z = 0.0;

% CRIANDO O INSCRITO EM "/scan"
scanNode = ros2node("/scanNode");
scanSub = ros2subscriber(scanNode, "/scan", @assignDist);

global right_dist
global front_right_dist

global indexOfRight
global indexOfFrontRight

indexOfRight = round((2.2689 - pi/2) / (0.0071));
indexOfFrontRight = round((2.2689 - pi/4) / (0.0071));

% Enviando dados.
rate = ros2rate(cmdNode, 10);   % 10 Hz
for i = 1:50
   send(cmdPub, cmdVels);
   waitfor(rate);
end

function assignDist(scanDataNew)
    global right_dist
    global front_right_dist
    global indexOfRight
    global indexOfFrontRight
    
    right_dist = scanDataNew.ranges(indexOfRight);
    front_right_dist = scanDataNew.ranges(indexOfFrontRight);
end 