// "방아쇠"가 당겨지면(메시지가 오면) 실행될 함수
// msg: 수신된 토픽 메시지 (e.g., 1 or 0)
// client: 두산 로봇 서비스에 연결된 "전화기"
void commandCallback(const std_msgs::Int32::ConstPtr& msg, ros::ServiceClient& client)
{
    // 서비스 메시지(총알) 준비
    dsr_msgs::MoveJoint srv;
    bool should_call_service = false; // 서비스 호출 여부 플래그

    // 메시지 값에 따라 다른 자세 설정
    if (msg->data == 1)
    {
        ROS_INFO("Trigger message '1' received. Preparing pose [90, 0, 90, 0, 90, -90]");
        srv.request.pos = {90.0, 0.0, 90.0, 0.0, 90.0, -90.0};
        should_call_service = true;
    }
    else if (msg->data == 0)
    {
        ROS_INFO("Trigger message '0' received. Preparing pose [-90, 0, 90, 0, 90, -90]");
        srv.request.pos = {-90.0, 0.0, 90.0, 0.0, 90.0, -90.0};
        should_call_service = true;
    }
    else
    {
        ROS_WARN("Received unsupported command: %d. Ignoring.", msg->data);
        should_call_service = false;
    }

    // 서비스 호출이 필요한 경우에만 실행
    if (should_call_service)
    {
        // 기타 파라미터 설정 (속도, 가속도 등)
        srv.request.vel = 30.0;
        srv.request.acc = 60.0;
        srv.request.mode = 0; // 0: MOVE_MODE_ABSOLUTE

        // 서비스 호출 ("발사")
        ROS_INFO("Calling move_joint service...");
        if (client.call(srv))
        {
            // 성공 응답 확인
            if(srv.response.success) {
                ROS_INFO("Service call successful: Robot moved.");
            } else {
                ROS_WARN("Service call reported failure.");
            }
        }
        else
        {
            ROS_ERROR("Failed to call service /dsr01a0912/motion/move_joint");
        }
    }
}