use opcua::server::prelude::{Config, DateTime, NodeId, Server, ServerConfig, Variable};
use std::path::PathBuf;

fn main() {
    rapid_clock();
}

fn rapid_clock() {
    opcua::console_logging::init();

    let mut server =
        Server::new(ServerConfig::load(&PathBuf::from("tests/resources/clock.conf")).unwrap());

    let namespace = {
        let address_space = server.address_space();
        let mut address_space = address_space.write();
        address_space.register_namespace("urn:rapid-clock").unwrap()
    };

    add_timed_variable(&mut server, namespace);

    server.run();
}

fn add_timed_variable(server: &mut Server, namespace: u16) {
    // These will be the node ids of the new variables
    let ticks_since_launch_node_id = NodeId::new(namespace, "100111");

    let address_space = server.address_space();

    // The address space is guarded so obtain a lock to change it
    {
        let mut address_space = address_space.write();

        let rapid_folder_id = address_space
            .add_folder("Rapid", "Rapid", &NodeId::objects_folder_id())
            .unwrap();

        let _ = address_space.add_variables(
            vec![Variable::new(
                &ticks_since_launch_node_id,
                "joint_positions",
                "joint_positions",
                vec![0.0f64; 6],
            )],
            &rapid_folder_id,
        );
    }

    {
        server.add_polling_action(10, move || {
            let mut address_space = address_space.write();
            let now = DateTime::now();
            let ticks_in_100_ns = now.ticks();
            let _ = address_space.set_variable_value(
                ticks_since_launch_node_id.clone(),
                vec![ticks_in_100_ns as f64; 6],
                &now,
                &now,
            );
        });
    }
}
