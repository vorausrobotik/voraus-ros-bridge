use socket2::{Domain, Socket, Type};
use std::io;
use std::net::SocketAddr;
use std::thread;
use std::time::{Duration, Instant};

mod helpers;

fn is_port_bound(port: u16) -> bool {
    let socket = match Socket::new(Domain::IPV4, Type::STREAM, None) {
        Ok(sock) => sock,
        Err(_) => return false,
    };

    // Set SO_REUSEADDR to allow for the server to bind while this function is testing
    if socket.set_reuse_address(true).is_err() {
        return false;
    }

    let addr = SocketAddr::from(([127, 0, 0, 1], port));

    // Attempt to bind the socket
    match socket.bind(&addr.into()) {
        Ok(_) => false, // Successfully bound, so port was not bound
        Err(ref e) if e.kind() == io::ErrorKind::AddrInUse => true, // Port is already bound
        Err(_) => false, // Other errors are treated as port not bound
    }
}

fn wait_for_function_to_pass<F>(function: F, timeout_ms: u64) -> Result<(), io::Error>
where
    F: Fn() -> bool,
{
    let start_time = Instant::now();
    let timeout_duration = Duration::from_millis(timeout_ms);
    let check_interval = Duration::from_millis(100);

    while Instant::now().duration_since(start_time) < timeout_duration {
        if function() {
            return Ok(());
        }
        thread::sleep(check_interval);
    }

    Err(io::Error::new(
        io::ErrorKind::TimedOut,
        "Expectation fn()==True timed out",
    ))
}

#[tokio::test]
async fn test_simple_server_binds_port() {
    // Start the server in the background
    helpers::opc_ua_publisher_single_linear::run_rapid_clock().await;

    let expected_server_port = 4855;
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
