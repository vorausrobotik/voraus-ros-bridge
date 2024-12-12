use socket2::{Domain, Socket, Type};
use std::ffi::OsString;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::time::{Duration, Instant};
use std::{env, io, thread};
use subprocess::{Popen, PopenConfig, Redirection};

pub fn is_port_bound(port: u16) -> bool {
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

pub fn wait_for_function_to_pass<F>(function: F, timeout_ms: u64) -> Result<(), io::Error>
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

pub struct ManagedRosBridge {
    process: Popen,
}

impl ManagedRosBridge {
    pub fn new(env: Option<Vec<(OsString, OsString)>>) -> subprocess::Result<Self> {
        // Start the ROS Brigde
        // We can't use ros2 run here because of https://github.com/ros2/ros2cli/issues/895
        let root_dir = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set");
        let mut path_to_executable = PathBuf::from(root_dir);
        path_to_executable
            .push("install/voraus-ros-bridge/lib/voraus-ros-bridge/voraus-ros-bridge");

        let process = Popen::create(
            &[path_to_executable],
            PopenConfig {
                stdout: Redirection::Pipe,
                stderr: Redirection::Pipe,
                detached: false,
                env,
                ..Default::default()
            },
        )?;

        Ok(ManagedRosBridge { process })
    }

    pub fn is_running(&mut self) -> bool {
        self.process.poll().is_none()
    }

    pub fn get_std_err(&mut self) -> Option<String> {
        let (_out, err) = self
            .process
            .communicate(None)
            .expect("Failed to capture output");
        err
    }

    pub fn terminate(&mut self) {
        let _ = self.process.terminate();
        let _ = self.process.wait();
    }
}

impl Drop for ManagedRosBridge {
    fn drop(&mut self) {
        if self.is_running() {
            self.terminate();
        }
    }
}
