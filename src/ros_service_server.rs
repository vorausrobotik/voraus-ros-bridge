pub fn handle_service(
    _request_header: &rclrs::rmw_request_id_t,
    request: voraus_ros_interfaces::srv::Voraus_Request,
) -> voraus_ros_interfaces::srv::Voraus_Response {
    println!("request: {} + {}", request.a, request.b);
    voraus_ros_interfaces::srv::Voraus_Response {
        sum: request.a + request.b,
    }
}
