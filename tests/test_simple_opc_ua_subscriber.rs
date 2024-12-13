pub mod common;
pub mod helpers;

use common::is_port_bound;
use common::wait_for_function_to_pass;
use helpers::opc_ua_test_server::OPCUATestServer;
use std::process::Stdio;
use std::sync::mpsc;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command as TokioCommand;
use tokio::time::{timeout, Duration as TokioDuration};

#[tokio::test]
async fn test_simple_subscriber_receives_data_changes() {
    let (assertion_tx, _assertion_rx) = mpsc::channel();
    let _server = OPCUATestServer::new(assertion_tx);

    let expected_server_port = 48401;
    wait_for_function_to_pass(|| is_port_bound(expected_server_port), 5000)
        .expect("Port was not bound within 5 seconds.");

    let mut client_process = TokioCommand::new("cargo")
        .arg("run")
        .env("RUST_LOG", "DEBUG")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("Failed to start client");

    let client_stdout = client_process
        .stderr
        .take()
        .expect("Failed to capture client stderr");
    let mut reader = BufReader::new(client_stdout).lines();

    let mut found_ticks_since_launch_changed_times = 0u32;
    let expected_changed_times = 10;

    let read_task = tokio::spawn(async move {
        while let Some(line) = reader
            .next_line()
            .await
            .expect("Failed to read line from client")
        {
            println!("Subscriber stderr: {}", line);
            if line.contains("Value = Array(Array { value_type: Double") {
                found_ticks_since_launch_changed_times += 1;
                if found_ticks_since_launch_changed_times == expected_changed_times {
                    return found_ticks_since_launch_changed_times;
                }
            }
        }
        found_ticks_since_launch_changed_times
    });

    let timeout_duration = TokioDuration::from_secs(5);
    match timeout(timeout_duration, read_task).await {
        Ok(result) => {
            found_ticks_since_launch_changed_times = result.expect("Failed to join read task");
        }
        Err(_) => {
            eprintln!("Test timed out");
        }
    }

    client_process.kill().await.expect("Failed to kill client");

    assert_eq!(
        found_ticks_since_launch_changed_times, expected_changed_times,
        "Client did not output enough value changes in time."
    );

}
