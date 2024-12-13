pub mod common;
pub mod helpers;

use std::sync::mpsc;

use common::is_port_bound;
use common::wait_for_function_to_pass;
use helpers::opc_ua_test_server::OPCUATestServer;

#[test]
fn test_simple_server_binds_port() {
    let (assertion_tx, _assertion_rx) = mpsc::channel();
    let _server = OPCUATestServer::new(assertion_tx);

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
