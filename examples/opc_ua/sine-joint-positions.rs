use opcua::server::prelude::{Config, DateTime, NodeId, Server, ServerConfig, Variable};
use std::f64::consts::PI;
use std::path::PathBuf;
use std::time::SystemTime;
use std::time::UNIX_EPOCH;

fn main() {
    sine_joint_positions();
}

fn sine_joint_positions() {
    opcua::console_logging::init();

    let mut server =
        Server::new(ServerConfig::load(&PathBuf::from("tests/resources/clock.conf")).unwrap());

    add_timed_variable_array(&mut server);

    server.run();
}

fn add_timed_variable_array(server: &mut Server) {
    // These will be the node id of the variable AxesMeasuredPosition
    let axes_measured_position = NodeId::new(1, 100111);

    let address_space = server.address_space();

    // The address space is guarded so obtain a lock to change it
    {
        let mut address_space = address_space.write();

        let axes_folder_id = address_space
            .add_folder("Axes", "Axes", &NodeId::objects_folder_id())
            .unwrap();

        let _ = address_space.add_variables(
            vec![Variable::new(
                &axes_measured_position,
                "AxesMeasuredPosition",
                "AxesMeasuredPosition",
                vec![0.0f64; 6],
            )],
            &axes_folder_id,
        );
    }

    {
        let omegas: [f64; 6] = [
            2.0 * PI * 0.25,
            2.0 * PI * 0.5,
            2.0 * PI,
            2.0 * PI * 1.5,
            2.0 * PI * 2.0,
            2.0 * PI * 2.5,
        ];

        server.add_polling_action(10, move || {
            let mut address_space = address_space.write();

            let timestamp_epoch = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
            let seconds =
                timestamp_epoch.as_secs() as f64 + timestamp_epoch.subsec_nanos() as f64 * 1e-9;

            let sine_values: Vec<f64> = omegas
                .iter()
                .map(|&omega| (omega * seconds).sin())
                .collect();

            let now = DateTime::now();
            let _ = address_space.set_variable_value(
                axes_measured_position.clone(),
                sine_values,
                &now,
                &now,
            );
        });
    }
}
