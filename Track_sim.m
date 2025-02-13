clear; close all; clc; % 작업공간 변수/객체 모두 삭제; 열려있는 figure(시각화 창) 닫기; 명령 창 내용 삭제 
rosshutdown; % ros 연결 해제
rosinit('http://localhost:11311') % ros 연결
tftree = rostf; % tftree 객체 생성 (전반적인 좌표계 변환을 위해???)
pause(3); % 멈춰

% Parameter (파라미터 설정)//============================================================

% roi(관심영역) : 순서대로 각 XYZ축의 최솟값과 최대값
roi = [0, 7, -7, 7, -2, 4];
%roi = [0, 7, -7, 7, -2, 4]; [0 15 -10 10 -2 4]

% cluster 여부 임계값
clusterThreshold = 0.1;

% 고려할 물체 크기 임계값 (임계값보다 작은 물체는 무시)
cuboidTreshold = 0.01; % (cone: 0.0215)

% waypoint에 근접함 여부 임계값 
waypointTreshold = 3; % 3미터 전에 새로운 waypoint 생성

% pp 제어기 객체 생성
pp=controllerPurePursuit;
pp.LookaheadDistance=1.5; % 유클리디안 거리, 단위 : m
pp.DesiredLinearVelocity=0.3; % 목표 직선 속도 설정 (경로를 안정적으로 추종하기 위해 일정 속도 유지), 단위 : m/s     %1.3
pp.MaxAngularVelocity = 0.4; % 최대 각속도 설정 (너무 급격히 회전하는 경우 방지), 단위 : rad/s    %1.6
%{
controllerPurePursuit
waypoint 따라가도록 만드는 제어기 객체 생성

구문
[vel,angvel] = controller(pose) % [선속도, 각속도] 계산값 받기
[vel,angvel,lookaheadpoint] = controller(pose) % 

입력 매개변수
pose : XY 좌표값, 쿼너니언

리턴
vel : 선속도
angvel : 각속도
lookaheadpoint : 전방주시지점 (x, y)

속성
DesiredLinearVelocity : 일저한 목표 선속도
LookaheadDistance : 전방주시거리
MaxAngularVelocity : 최대 각속도
Waypoints : 
%}


%init (초기화 설정)//==================================================================

% 차량 체크포인트들
waypoints = [];


% marker ID 넘버
% 지나온 경로 시각화할 마커 넘버
markerIdPath = 0;
% 인식한 각 클러스터 시각화할 마커 넘버
markerIdClusters = 0;


% lidar 파라미터 객체 생성 (시각화에 필요)
params = lidarParameters('OS1Gen1-64',512);

% lidar 토픽 - sub 객체 생성
lidarSub = rossubscriber('/lidar/points', "DataFormat", "struct");
%detect_sub_l = rossubscriber('yolov5/cob_detections_l', 'cob_perception_msgs/DetectionArray');
%detect_sub_r = rossubscriber('yolov5/cob_detections_r', 'cob_perception_msgs/DetectionArray');

% lidar 토픽 - pub & msg 객체 생성
[pubClusters, markerArrayMsg] = rospublisher('/clusters_marker', 'visualization_msgs/MarkerArray',DataFormat='struct');
% '/clusters_marker' 토픽 생성 -> 'visualization_msgs/MarkerArray' msg 타입 

% 경로 생성 토픽 - pub 객체 생성
pubPath = rospublisher('/path_marker', 'visualization_msgs/Marker','DataFormat','struct');
% '/path_marker' 토픽 생성 -> 'visualization_msgs/Marker' msg 타입 

% gazebo 토픽 - sub 객체 생성
modelSub = rossubscriber('/gazebo/model_states',"DataFormat", "struct");
% '/gazebo/model_states' 토픽 생성 -> 모델 정보 제공



% 탐지된 객체 정보 서비스 - client 객체 생성
client = rossvcclient('/Activate_yolo','cob_object_detection_msgs/DetectObjects',DataFormat='struct'); 
% '/Activate_yolo' 서비스 생성 ->'cob_object_detection_msgs/DetectObjects'msg타입

% service 요청 메시지 객체 생성
request_l = rosmessage(client);
request_l.ObjectName.Data = 'left'; % 'left' 데이터 찾기
request_r = rosmessage(client);
request_r.ObjectName.Data = 'right'; % 'left' 데이터 찾기

% 카메라 - 좌표계 변환 행렬 정보 불러오기
load("camera1_tform.mat");
camera1_tform = tform;
tformCamera1 = invert(camera1_tform); 

load("camera2_tform.mat");
camera2_tform = tform; 
tformCamera2 = invert(camera2_tform);

% 참고
%{
camera_tform.mat 1, 2
.mat파일 안에 tform 객체 하나 있음.
1x1 rigidtform3d -> 1개의 강체에 대한 강체 변환 정보를 포함한다라는 의미

속성&밸류 in tform
tform.A : [4x4 double] -> 강체 변환 행렬
tform.R : [3x3 double] -> 회전행렬
tform.Translation : [1x3 double] -> 평행이동 행렬
tform.Dimensionality : 3
%}

% 카메라 - 내부 파라미터 정보 불러오기
load("camera1_Params.mat");
cameraParam_r = cameraParams;
load("camera2_Params.mat");
cameraParam_l = cameraParams;

% 참고
%{
camera1_Params.mat 1, 2
.mat파일 안에 cameraParams 객체 하나 있음.
1x1 cameraParameters

속성&밸류 in cameraParameters
개많음.
Intrinsic, Extrinsic, Camera Lens Distortion...등
이따가 알아가보도록 하자.
%}


% figure;

while true % ctrl + c to stop
    tic; % 타임 카운트 
    
    % 현재 차량 위치 및 방향 tftree에 업데이트
    vehiclePose = updateVehiclePose_GT(modelSub, tftree);
    
    % 남은 waypoint 없음 or 월드좌표계에서 목적지와 현재 위치가 거의 근접함
    if isempty(pp.Waypoints) || norm(worldWaypoints(end,:)-[vehiclePose(1), vehiclePose(2)]) < waypointTreshold  % Considering only x and y for the distance
        disp("Make new waypoints"); % 메시지 출력

        try
            % lidar points msg 받기
            lidarData = receive(lidarSub); % lidarSub : /lidar/points Topic Subscriber

            % client가 request(YOLO msg) 요청
            % 카메라 좌표계 l/r (Bounding Box 저장)
            bboxData_r = call(client, request_r);
            bboxData_l = call(client, request_l);
            
            % 라이다 데이터 전처리
            % 비지상 부분 포인트 클라우드만 -> 노이즈 제거 후 리턴
            roiPtCloud = preprocess_lidar_data(lidarData, params, roi);
            
            % l/r 카메라 좌표계 -> 각각 y/b로 나누기
            [y_coneBboxs_l, b_coneBboxs_l] = extractConesBboxs(bboxData_l.ObjectList);
            [y_coneBboxs_r, b_coneBboxs_r] = extractConesBboxs(bboxData_r.ObjectList);
            
            % 라이다 좌표계 l/r (카메라 좌표계를 변환)
            [bboxesLidar_l,~,boxesUsed_l] = bboxCameraToLidar([y_coneBboxs_l; b_coneBboxs_l],roiPtCloud,cameraParam_l,tformCamera2,'ClusterThreshold',clusterThreshold);
            [bboxesLidar_r,~,boxesUsed_r] = bboxCameraToLidar([y_coneBboxs_r; b_coneBboxs_r],roiPtCloud,cameraParam_r,tformCamera1,'ClusterThreshold',clusterThreshold);
            % bboxCameraToLidar()
            %{
            입력인수
            [y_coneBboxs_l; b_coneBboxs_l] (배열 결합)
            roiPtCloud
            cameraParam_l
            tformCamera2
            'ClusterThreshold'
            clusterThreshold

            출력인수
            bboxesLidar_l : 라이다 좌표계의 Bounding Box 배열
            ~ : 무시 (점들 인덱스 데이터)
            boxesUsed_l : 사용된 2차원 Bounding Box 배열 -> 성공 여부 확인용
            %}

            % l/r 라이다 좌표계 -> 각각 y/b로 나누기
            [y_coneBboxesLidar_l, b_coneBboxesLidar_l] = splitConesBboxes(y_coneBboxs_l,bboxesLidar_l,boxesUsed_l);
            [y_coneBboxesLidar_r, b_coneBboxesLidar_r] = splitConesBboxes(y_coneBboxs_r,bboxesLidar_r,boxesUsed_r);
            
            % cone XY좌표 추출 (cuboidTreshold보다 부피가 큰)
            % b -> inner cone / y -> outer cone
            innerConePosition = extractConePositions(cuboidTreshold, b_coneBboxesLidar_l, b_coneBboxesLidar_r);
            outerConePosition = extractConePositions(cuboidTreshold, y_coneBboxesLidar_l, y_coneBboxesLidar_r);
            
            % 중복된 행 지우기 (고유한 행으로만 구성)
            innerConePosition = unique_rows(innerConePosition);
            outerConePosition = unique_rows(outerConePosition);
            
            % scatter plot으로 시각화
            hold off; % 기존 그래프 지우기
            scatter(innerConePosition(:,1),innerConePosition(:,2),'blue'); % 출력
            hold on; % 기존 그래프 유지 (파란콘 scatter 유지한 채 노란콘 scatter plot)
            scatter(outerConePosition(:,1),outerConePosition(:,2),'red');
            
            % 행 길이 맞추기
            [innerConePosition, outerConePosition] = match_array_lengths(innerConePosition, outerConePosition);
            
            % waypoint 생성
            waypoints = generate_waypoints_del(innerConePosition, outerConePosition);
            worldWaypoints = transformWaypointsToOdom(waypoints, vehiclePose);
            
            % 들로네 삼각분할 (Delaunay triangulation)
            %{
            점들을 이어 삼각형을 만들 때, 이 삼각형들의 내각의 최솟값이 최대가 되도록 하는 분할.
            삼각형은 정삼각형에 가까울수록, 내각의 최솟값이 60도에 근사하게 된다.
            %}

            [markerArrayMsg, markerIdClusters] = generate_clusters_marker(innerConePosition, outerConePosition, 'base_footprint', markerIdClusters);
            send(pubClusters, markerArrayMsg);

            [pathMarkerMsg, markerIdPath] = generate_path_marker(worldWaypoints, 'erp42_base', markerIdPath);
            send(pubPath, pathMarkerMsg);
     
            pp.Waypoints = worldWaypoints;
        catch
            disp("Fail to make new waypoints");
            % For check the exact clustering box//========================
            % pcshow(roiPtCloud);
            % xlim([0 10])
            % ylim([-5 5])
            % 
            % hold on;
            % showShape('cuboid',y_coneBboxesLidar_r,'Opacity',0.5,'Color','green');
            % showShape('cuboid',b_coneBboxesLidar_r,'Opacity',0.5,'Color','red');
            % showShape('cuboid',y_coneBboxesLidar_l,'Opacity',0.5,'Color','blue');
            % showShape('cuboid',b_coneBboxesLidar_l,'Opacity',0.5,'Color','red');
            % return;
            continue; % 다음 while문 반복으로 넘어감
        end
    end
    
    % 경로 추종
    [v, w] = pp(vehiclePose); % 선속도, 각속도 받기
    [pub, msg] = publish_twist_command(v, w, '/cmd_vel'); % twist msg 생성 ('/cmd_vel' 토픽 생성)
    send(pub, msg); % 속도 명령 보내기
    toc; % 시간 카운트 종료
end



%%
function vehiclePose = updateVehiclePose(ekfStatesSub, tftree)
    %vehiclePoseOdom = getVehiclePose(tftree, 'ackermann_steering_controller/odom', 'base_footprint');
    %vehiclePose = vehiclePoseOdom;

    ekfStatesMsg = receive(ekfStatesSub);
   
    robotPose = ekfStatesMsg.Pose.Pose;
    quat = [robotPose.Orientation.W, robotPose.Orientation.X, robotPose.Orientation.Y, robotPose.Orientation.Z];
    euler = quat2eul(quat);
    yaw = euler(1);
    vehiclePoseekf=[robotPose.Position.X; robotPose.Position.Y; yaw];

    % TF 메시지 생성 및 설정
    tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
    tfStampedMsg.ChildFrameId = 'base_link';
    tfStampedMsg.Header.FrameId = 'erp42_base';
    tfStampedMsg.Header.Stamp = rostime('now');
    tfStampedMsg.Transform.Translation.X = vehiclePoseekf(1);
    tfStampedMsg.Transform.Translation.Y = vehiclePoseekf(2);
    tfStampedMsg.Transform.Rotation.Z = sin(vehiclePoseekf(3)/2);
    tfStampedMsg.Transform.Rotation.W = cos(vehiclePoseekf(3)/2);

    % TF 브로드캐스팅
    sendTransform(tftree, tfStampedMsg);

    vehiclePose = vehiclePoseekf;

end

% 차량 위치와 방향 업데이트
function vehiclePose = updateVehiclePose_GT(modelSub, tftree)
    
    % 현재 차량 위치, 방향 정보 저장
    modelMsg = receive(modelSub); % modelSub sub객체로 gazebo 정보 메시지를 modelMsg에 저장
    robotIndex = find(strcmp(modelMsg.Name, 'robot'));  % gazebo에 존재하는 객체들 이름이 저장되어있는 문자열 중 'robot'이름의 인덱스 번호 저장
    robotPose = modelMsg.Pose(robotIndex); % robot 객체의 위치 및 방향 정보 저장
    
    quat = [robotPose.Orientation.W, robotPose.Orientation.X, robotPose.Orientation.Y, robotPose.Orientation.Z]; % 4차원 벡터의 쿼터니언 저장
    euler = quat2eul(quat); % 쿼터니언 -> 오일러 
    yaw = euler(1); % yaw(z축 기준 회전각) 저장
    
    vehiclePose = [robotPose.Position.X ; robotPose.Position.Y; yaw]; % [x;y;yaw] -> 차량의 현재 위치(X,Y)와 방향(yaw) 저장

    %{
    modelMsg.Pose()
    Position : x y z 값 
    Orientation : 쿼터니언(회전관련 데이터)

    quat2eul()
    쿼터니언 -> 오일러 각으로 변환
    return 값 : [yaw, pitch, roll] (기본적으로 ZXY순서로 반환)
    yaw : z축을 기준으로 회전한 각도
    %}


    % TF 메시지 생성 및 설정
    tfStampedMsg = rosmessage('geometry_msgs/TransformStamped'); % TF 메시지 객체 생성
    tfStampedMsg.ChildFrameId = 'base_link'; % 자식좌표계 'base_link'로 명명
    tfStampedMsg.Header.FrameId = 'erp42_base'; % 부모 좌표계 'erp42_base'로 명명
    tfStampedMsg.Header.Stamp = rostime('now'); % 타임스탬프(메시지 생성 시간) 현재 시간으로 설정 
    tfStampedMsg.Transform.Translation.X = vehiclePose(1); % 현재 차량의 X좌표 저장
    tfStampedMsg.Transform.Translation.Y = vehiclePose(2); % 현재 차량의 Y좌표 저장
    tfStampedMsg.Transform.Rotation.Z = sin(vehiclePose(3)/2); % yaw -> 쿼터니언 변환하고 저장
    tfStampedMsg.Transform.Rotation.W = cos(vehiclePose(3)/2); % tf시스템에서는 회전 정보는 쿼터니언으로 표현해야함


    % TF 브로드캐스팅 (TF 업데이트)
    sendTransform(tftree, tfStampedMsg); % 네트워크에 브로드캐스트
    
    %{
    TransformStamped
    변환(회전 및 평행이동)에 대한 내용을 담는 메시지 타입
    
    속성
    MessageType(문자열)
    Header(객체) : Message Type(해더객체가 사용하는 메시지 타입 문자열), seq(변환 메시지 순서), timestamp, FrameId(변환할 기존 좌표계 이름)
    ChildFrameID(문자열) : 변환을 적용할 자식 좌표계 이름
    Transform(객체) : 평행이동 벡터와 회전 쿼터니언 정보 담긴 메시지
    %}
end

% cuboidTreshold보다 큰 cone 좌표 추출
function conePosition = extractConePositions(cuboidTreshold, coneBboxesLidar_l, coneBboxesLidar_r)
    % Extract xlen, ylen, zlen from the bounding boxes
    % 부피 계산 (바운딩 박스 각 행 [x, y, z, xlen, ylen, zlen]에서 xlen, ylen, zlen 사용)
    volumes_l = prod(coneBboxesLidar_l(:, 4:6), 2); % 4~6번째 열의 곱
    volumes_r = prod(coneBboxesLidar_r(:, 4:6), 2);

    % Find indices where volumes are bigger than cuboidThreshold
    indices_l = volumes_l > cuboidTreshold; %T/F data (mx1 matrix)
    indices_r = volumes_r > cuboidTreshold;

    % l/r의 X,Y(좌표) 하나로 합치기 (세로로 합침)
    conePosition = [[coneBboxesLidar_l(indices_l, 1)+0.6, coneBboxesLidar_l(indices_l, 2)];
                    [coneBboxesLidar_r(indices_r, 1)+0.6, coneBboxesLidar_r(indices_r, 2)]];
    % row : l/r의 합친 row 개수 / column : XY
end

% bboxesLidar에서 y_cone와 b_cone의 bbox 분류
function [y_coneBboxesLidar, b_coneBboxesLidar] = splitConesBboxes(y_coneBboxs,bboxesLidar,boxesUsed)
    % y_cone의 유효한 Bounding Box 총 개수
    numY_cone = sum(boxesUsed(1:size(y_coneBboxs,1)));
    % size(y_coneBboxs,1)) : y_coneBboxs의 행의 수 (만약 2 이면, 열의 수)
    
    % bboxesLidar에서 y_cone와 b_cone의 bbox 분류
    y_coneBboxesLidar = bboxesLidar(1:numY_cone, :);
    b_coneBboxesLidar = bboxesLidar(numY_cone+1:end, :);
end



function odomWaypoints = transformWaypointsToOdom(waypoints, vehiclePoseInOdom)
    % Initialize transformed waypoints
    odomWaypoints = zeros(size(waypoints));

    % Extract the vehicle's yaw angle
    theta = vehiclePoseInOdom(3);

    % Create the 2D rotation matrix
    R = [cos(theta), -sin(theta);
         sin(theta), cos(theta)];

    % Transform each waypoint
    for i = 1:size(waypoints,1)
        % Rotate the waypoint considering the vehicle's yaw
        rotatedPoint = R * waypoints(i,:)';

        % Translate considering the vehicle's position in the odom frame
        odomWaypoints(i,:) = rotatedPoint' + vehiclePoseInOdom(1:2)';
    end
end

% 노랑/파랑 콘 Bounding Box 추출
function [y_coneBboxs, b_coneBboxs] = extractConesBboxs(bboxData)
    % bboxData -> bboxData_l.ObjectList / bboxData_r.ObjectList
    % yolo로 탐지된 모든 객체 리스트

    % BoundingBoxes_ 대신 Detections의 길이로 메모리 공간을 미리 할당 (element 개수)
    numBboxes = numel(bboxData.Detections);

    % y_cone 및 b_cone에 대한 임시 저장 공간
    temp_y_coneBboxs = zeros(numBboxes, 4);
    temp_b_coneBboxs = zeros(numBboxes, 4);
    % size : 'numBboxes' x 4
    % 탐지한 모든 콘 개수만큼 저장 공간 넉넉히 세팅
    
    % 콘 개수 카운트
    y_count = 0;
    b_count = 0;
    
    % Bounding Box 처리 (1 ~ numBboxes까지)
    for i = 1:numBboxes
        currentBbox = bboxData.Detections(i, 1).Mask.Roi;
        
        % 변경된 데이터 형식에 따라 BoundingBoxes_ 대신 Mask.Roi 사용
        x = currentBbox.X;
        y = currentBbox.Y;
        w = currentBbox.Width;
        h = currentBbox.Height;
        
        % 라벨 분류 (노랑 /파랑)
        if strcmp(bboxData.Detections(i, 1).Label, 'y_cone')
            y_count = y_count + 1;
            temp_y_coneBboxs(y_count, :) = [x, y, w, h]; 
            % y_count row번째 현재 노란 콘의 [x, y, w, h] 저장 

        else % 'b_cone'
            b_count = b_count + 1;
            temp_b_coneBboxs(b_count, :) = [x, y, w, h];
            % b_count번째 row에 현재 파란 콘의 [x, y, w, h] 저장
        end
    end

    % 최종 결과
    y_coneBboxs = temp_y_coneBboxs(1:y_count, :); % 노란 콘 개수만큼
    b_coneBboxs = temp_b_coneBboxs(1:b_count, :); % 파란 콘 개수만큼
end

function vehiclePose = getVehiclePose(tree, sourceFrame, targetFrame)
    % This function returns the pose of the vehicle in the odom frame.

    % Check if the frames are available in the tree
    if ~any(strcmp(tree.AvailableFrames, sourceFrame))
        error('Source frame is not available in the tree');
    end
    if ~any(strcmp(tree.AvailableFrames, targetFrame))
        error('Target frame is not available in the tree');
    end

    % Wait for the transformation to be available
    waitForTransform(tree, sourceFrame, targetFrame); 

    % Get the transformation
    tf = getTransform(tree, sourceFrame, targetFrame);

    % Extract the vehicle's pose
    trans = [tf.Transform.Translation.X;
             tf.Transform.Translation.Y];

    quat = [tf.Transform.Rotation.W;
            tf.Transform.Rotation.X;
            tf.Transform.Rotation.Y;
            tf.Transform.Rotation.Z];

    eul = quat2eul(quat');  % Get the euler angles in ZYX order (yaw, pitch, roll)

    vehiclePose = [trans; eul(1)];  % Vehicle's pose in [x, y, theta(yaw)]
end



% 라이다 데이터 -> 포인트 클라우드로 변환
% 비지상 부분 포인트 클라우드만 -> 노이즈 제거 후 리턴
function roiPtCloud = preprocess_lidar_data(lidarData, params, roi)
    xyzData = rosReadXYZ(lidarData); % n개의 라이다 데이터 xyz 좌표 추출 -> nx3 행렬
    ptCloud = pointCloud(xyzData); % pointCloud객체로 반환

    ptCloudOrg = pcorganize(ptCloud, params); % 정렬 포인트 클라우드 변환 (pointCloud객체로 리턴)
    
    % 지상/비지상 부분으로 포인트 클라우드 인덱스 나누기  
    groundPtsIdx = segmentGroundFromLidarData(ptCloudOrg); % 지상 부분의 포인트 클라우드 인덱스

    nonGroundPtCloud = select(ptCloudOrg, ~groundPtsIdx, 'OutputSize', 'full'); % 비지상 부분의 포인트 클라우드
    indices = findPointsInROI(nonGroundPtCloud, roi); % 비지상 부분 중 roi(관심영역) 내 포인트만 인덱스 저장
    roiPtCloud = select(nonGroundPtCloud, indices); % roi내 비지상 부분 포인트 클라우드 리턴
    roiPtCloud = pcdenoise(roiPtCloud, 'PreserveStructure', true); % 노이즈 제거 (MxNx3 정렬 포인트 클라우드 유지한 채 리턴)
end

function [centers, innerConePosition, outerConePosition] = process_clusters(roiPtCloud)
    [labels, numClusters] = pcsegdist(roiPtCloud, 0.3);

    xData = roiPtCloud.Location(:,1);
    yData = roiPtCloud.Location(:,2);

    clf;
    hold on;
    centers = [];
    innerConePosition = [];
    outerConePosition = [];
    for i = 1:numClusters
        idx = labels == i;
        clusterPoints = [xData(idx), yData(idx), roiPtCloud.Location(idx,3)];

        if size(clusterPoints, 1) >= 20
            [~, maxZIdx] = max(clusterPoints(:,3));
            center = clusterPoints(maxZIdx, 1:2);
            centers = [centers; center];

            if center(2)<0
                innerConePosition=[innerConePosition; center(1), center(2)];
            else
                outerConePosition=[outerConePosition; center(1), center(2)];
            end
            scatter(center(1), -center(2), "red","filled");
        end
    end
end

% 중복된 행 지우기 (고유한 행으로만 구성)
function uniqueArray = unique_rows(array)
    [~, uniqueIdx] = unique(array, 'rows'); % 고유한 행 인덱스 저장(값은 무시, 행 단위로 검사)
    uniqueArray = array(uniqueIdx, :); % 고유한 행으로만 구성
    uniqueArray = sortrows(uniqueArray); % 오름차순 정렬
end

% 파란 콘, 노란 콘 XY좌표 행렬 row 길이 맞추
function [out1, out2] = match_array_lengths(arr1, arr2)
    len1 = size(arr1, 1); % Get the number of rows
    len2 = size(arr2, 1); % Get the number of rows

    if len1 > len2
        out1 = arr1(1:len2, :); % Keep only the first len2 rows
        out2 = arr2;
    elseif len2 > len1
        out1 = arr1;
        out2 = arr2(1:len1, :); % Keep only the first len1 rows
    else
        out1 = arr1;
        out2 = arr2;
    end
end

function waypoints = generate_waypoints_del(innerConePosition, outerConePosition)
    [m,nc] = size(innerConePosition); % size of the inner/outer cone positions data

    kockle_coords = zeros(2*m,nc); % initiate a P matrix consisting of inner and outer coordinates
    kockle_coords(1:2:2*m,:) = innerConePosition;
    kockle_coords(2:2:2*m,:) = outerConePosition; % merge the inner and outer coordinates with alternate values
    xp = []; % create an empty numeric xp vector to store the planned x coordinates after each iteration
    yp = []; 

    
    interv=size(innerConePosition,1)*2;
    %step 1 : delaunay triangulation
    tri=delaunayTriangulation(kockle_coords);%
    pl=tri.Points;
    cl=tri.ConnectivityList;
    [mc, nc]=size(pl);
		    
    % inner and outer constraints when the interval is even
    if rem(interv,2) == 0
     cIn = [2 1;(1:2:mc-3)' (3:2:(mc))'; (mc-1) mc];
     cOut = [(2:2:(mc-2))' (4:2:mc)'];
    else
    % inner and outer constraints when the interval is odd
    cIn = [2 1;(1:2:mc-2)' (3:2:(mc))'; (mc-1) mc];
    cOut = [(2:2:(mc-2))' (4:2:mc)'];
    end
    
    %step 2 : 외부 삼각형 거
    C = [cIn;cOut];
    TR=delaunayTriangulation(pl,C);
    TRC=TR.ConnectivityList;
    TL=isInterior(TR);
    TC =TR.ConnectivityList(TL,:);
    [~, pt]=sort(sum(TC,2));
    TS=TC(pt,:);
    TO=triangulation(TS,pl);
		    
	%step 3 : 중간 waypoint 생성
    xPo=TO.Points(:,1);
    yPo=TO.Points(:,2);
    E=edges(TO);
    iseven=rem(E,2)==0;
    Eeven=E(any(iseven,2),:);
    isodd=rem(Eeven,2)~=0;
    Eodd=Eeven(any(isodd,2),:);
    xmp=((xPo((Eodd(:,1))) + xPo((Eodd(:,2))))/2);
    ymp=((yPo((Eodd(:,1))) + yPo((Eodd(:,2))))/2);
    Pmp=[xmp ymp];
    waypoints = Pmp;

    % 	    %step 4 : waypoint 보간
    %distancematrix = squareform(pdist(Pmp));
    %distancesteps = zeros(length(Pmp)-1,1);
    %for j = 2:length(Pmp)
    %     distancesteps(j-1,1) = distancematrix(j,j-1);
    % end
    % totalDistance = sum(distancesteps); % total distance travelled
    % distbp = cumsum([0; distancesteps]); % distance for each waypoint
    % gradbp = linspace(0,totalDistance,100);
    % xq = interp1(distbp,xmp,gradbp,'spline'); % interpolate x coordinates
    % yq = interp1(distbp,ymp,gradbp,'spline'); % interpolate y coordinates
    % xp = [xp xq]; % store obtained x midpoints after each iteration
    % yp = [yp yq]; % store obtained y midpoints after each iteration
    % %
	% 	    step 5 : 최종 waypoint 생성
    %waypoints=[xp', yp'];
end

function waypoints = generate_waypoints(innerConePosition, outerConePosition)
	%go_traingulation
    
    [m,nc] = size(innerConePosition); % size of the inner/outer cone positions data
    kockle_coords = zeros(2*m,nc); % initiate a P matrix consisting of inner and outer coordinates
    kockle_coords(1:2:2*m,:) = innerConePosition;
    kockle_coords(2:2:2*m,:) = outerConePosition;
    xp=[];
    yp=[];

    midpoints=zeros(size(kockle_coords, 1)-1 , size(kockle_coords,2));

    for i=1:size(kockle_coords, 1) -1
        midpoints(i,1)=(kockle_coords(i,1)+kockle_coords(i+1,1)) /2;
        midpoints(i,2)=(kockle_coords(i,2)+kockle_coords(i+1,2)) /2;
    end
    waypoints = midpoints;
    
    % distancematrix = squareform(pdist(midpoints));
    % distancesteps = zeros(length(midpoints)-1,1);
    % for j = 2:length(midpoints)
    %     distancesteps(j-1,1) = distancematrix(j,j-1);
    % end
    % totalDistance = sum(distancesteps); % total distance travelled
    % distbp = cumsum([0; distancesteps]); % distance for each waypoint
    % gradbp = linspace(0,totalDistance,100);
    % xq = interp1(distbp,midpoints(:,1),gradbp,'spline'); % interpolate x coordinates
    % yq = interp1(distbp,midpoints(:,2),gradbp,'spline'); % interpolate y coordinates
    % xp = [xp xq]; % store obtained x midpoints after each iteration
    % yp = [yp yq]; % store obtained y midpoints after each iteration
    % 
    % waypoints=[xp', yp'];
end

function [markerArrayMsg, markerID] = generate_clusters_marker(b_coneCluster, y_coneCluster, frameId, markerID)
    % Concatenate the clusters for easier looping
    combinedClusters = [b_coneCluster; y_coneCluster];
    clusterColors = [repmat([0 0 1], size(b_coneCluster, 1), 1); % Blue for b_coneCluster
                     repmat([1 1 0], size(y_coneCluster, 1), 1)]; % Yellow for y_coneCluster

    markerArrayMsg = rosmessage('visualization_msgs/MarkerArray','DataFormat','struct');
    
    for i = 1:height(combinedClusters)
        markerMsg = rosmessage('visualization_msgs/Marker','DataFormat','struct');
        markerMsg.Header.FrameId = frameId;
        markerMsg.Id = int32(markerID); % Cast 'markerID' to int32
        markerID = markerID + 1;  % Increment markerID by 1 for each new marker
        markerMsg.Type = int32(3);
        markerMsg.Action = int32(0);
        markerMsg.Pose.Position.X = double(combinedClusters(i,1));
        markerMsg.Pose.Position.Y = double(combinedClusters(i,2));
        markerMsg.Pose.Position.Z = 0;
        markerMsg.Pose.Orientation.W = 1.0;
        markerMsg.Scale.X = 0.3;
        markerMsg.Scale.Y = 0.3;
        markerMsg.Scale.Z = 0.5;
        
        % Set Color
        markerMsg.Color.R = single(clusterColors(i, 1));
        markerMsg.Color.G = single(clusterColors(i, 2));
        markerMsg.Color.B = single(clusterColors(i, 3));
        markerMsg.Color.A = single(0.5);
        
        markerArrayMsg.Markers(i) = markerMsg;
    end
end


function [markerMsg, markerID] = generate_path_marker(waypoints, frameId, markerID)
    markerMsg = rosmessage('visualization_msgs/Marker','DataFormat','struct');
    markerMsg.Header.FrameId = frameId;
    markerMsg.Id = int32(markerID);  % Cast 'markerID' to int32
    markerID = markerID + 1;  % Increment markerID by 1 for each new marker
    markerMsg.Type = int32(4); % LINE_STRIP
    markerMsg.Action = int32(0);
    markerMsg.Pose.Orientation.W = 1.0;
    markerMsg.Scale.X = 0.05;  % Specify a suitable scale
    markerMsg.Color.R = single(1.0); % red
    markerMsg.Color.G = single(0.0); % green
    markerMsg.Color.B = single(0.0); % blue
    markerMsg.Color.A = single(1.0); % alpha

    % Add the waypoints to the points array of the marker
    for i = 1:size(waypoints, 1)
        point = rosmessage('geometry_msgs/Point','DataFormat','struct');
        point.X = double(waypoints(i, 1));
        point.Y = double(waypoints(i, 2));
        point.Z = 0;
        markerMsg.Points(i) = point;
    end
end


% Twist 메시지 생성 (속도 지정)
function [pub, msg] = publish_twist_command(v, w, topicName)
    pub = rospublisher(topicName, 'geometry_msgs/Twist','DataFormat','struct'); % twist 메시지 pub 객체 생성
    msg = rosmessage(pub);
    msg.Linear.X = v; % 선속도
    msg.Angular.Z = w; % 각속도
end
