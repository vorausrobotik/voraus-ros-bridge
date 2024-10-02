pub mod common;
pub mod helpers;

use common::is_port_bound;
use common::wait_for_function_to_pass;

#[tokio::test]
async fn test_simple_server_binds_port() {
    // Start the server in the background
    helpers::opc_ua_publisher_single_linear::run_rapid_clock().await;

    let expected_server_port = 48401;
    let res = wait_for_function_to_pass(|| is_port_bound(expected_server_port), 5000);

    match res {
        Ok(_) => println!("Port was bound within 5 seconds."),
        Err(e) => {
            panic!(
                "Failed to assert port {} binding within the timeout period: {}",
                expected_server_port, e
            );
        }
    }
}
