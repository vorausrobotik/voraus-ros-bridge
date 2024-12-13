use std::{
    sync::{mpsc, Arc},
    time::Duration,
};

use common::{wait_for_function_to_pass, ManagedRosBridge};
use helpers::opc_ua_test_server::OPCUATestServer;
use std_srvs::srv::Empty;
use voraus_interfaces::srv::MoveJoints;

pub mod common;
pub mod helpers;

macro_rules! make_testcase_method {
    ($value:expr, $service_type:path, $testname:ident) => {
        #[test]
        fn $testname() {
            let (assertion_tx, assertion_rx) = mpsc::channel();
            let _server = OPCUATestServer::new(assertion_tx);

            let mut _bridge_process =
                ManagedRosBridge::new(None).expect("Failed to start subprocess");

            let service_caller = Arc::new(
                helpers::ros_service_caller::ServiceCaller::<$service_type>::new(&format!(
                    "/voraus_bridge_node/{}",
                    $value
                ))
                .unwrap(),
            );

            // TODO: Figure out why this takes almost 10 s [RR-836]
            service_caller.start();
            assert!(*service_caller.number_of_calls.lock().unwrap() == 0);
            service_caller.call();

            wait_for_function_to_pass(
                || *service_caller.number_of_calls.lock().unwrap() == 1,
                5000,
            )
            .unwrap();

            wait_for_function_to_pass(
                || {
                    let received = assertion_rx
                        .recv_timeout(Duration::from_millis(10))
                        .unwrap();
                    received.contains($value)
                },
                5000,
            )
            .unwrap();
        }
    };
}

make_testcase_method!(
    "impedance_control/enable",
    Empty,
    test_enable_impedance_control
);
make_testcase_method!(
    "impedance_control/disable",
    Empty,
    test_disable_impedance_control
);
make_testcase_method!("move_joints", MoveJoints, test_move_joints);
